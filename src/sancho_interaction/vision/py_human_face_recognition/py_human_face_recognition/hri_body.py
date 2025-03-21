import math
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import  JointState
from nav_msgs.msg import Odometry

from human_face_recognition_msgs.msg import BodyOrientation, BodyMovement
from human_face_recognition_msgs.srv import GetBool
from .hri_bridge import HRIBridge


class HRIBodyNode(Node):

    def __init__(self, hri_body):
        super().__init__("hri_body")

        self.head_pan = 0
        self.head_tilt = 0
        self.body_z = 0
        self.body_x = 0
        self.block_body_rotation = False
        self.block_body_movement = False
        
        self.body_alive = False
        self.last_motor_callback = time.time()

        self.sub_block_orientation = self.create_subscription(Bool, "/robot/body/block_orientation", self.block_orientation_callback, 1)
        self.sub_block_movement = self.create_subscription(Bool, "/robot/body/block_movement", self.block_movement_callback, 1)
        self.sub_orientation = self.create_subscription(BodyOrientation, "/robot/orientation", self.orientation_callback, 1)
        self.sub_movement = self.create_subscription(BodyMovement, "/robot/body/movement", self.movement_callback, 1)
        
        self.sub_detection_data = self.create_subscription(JointState, "/wxxms/joint_states", self.head_position_callback, 1)
        self.sub_odometry = self.create_subscription(Odometry, "pose", self.odometry_callback, 1)
        
        self.cmd_vel = self.create_publisher(Twist, "/cmd_vel", 1)
        self.pub_rotating = self.create_publisher(Bool, "/robot/rotating", 1)
        self.pub_moving = self.create_publisher(Bool, "/robot/body/moving", 1)

        self.is_orientation_blocked_serv = self.create_service(GetBool, "/robot/body/is_orientation_blocked", self.is_orientation_blocked)
        self.is_movement_blocked_serv = self.create_service(GetBool, "/robot/body/is_movement_blocked", self.is_movement_blocked)

        self.hri_body = hri_body
        self.br = HRIBridge()

        self.get_logger().info("Body inicializado correctamente")

    def is_orientation_blocked(self, request, response):
        response.value = Bool(data=self.block_body_rotation)
        return response

    def is_movement_blocked(self, request, response):
        response.value = Bool(data=self.block_body_movement)
        return response

    def block_orientation_callback(self, msg):
        if self.block_body_rotation != msg.data:
            self.get_logger().info("ROTACION DE CUERPO " + ("" if msg.data else "DES") + "BLOQUEADO")
            if not msg.data: # Reiniciamos los targets
                self.hri_body.target_z = self.body_z

        self.block_body_rotation = msg.data

    def block_movement_callback(self, msg):
        if self.block_body_movement != msg.data:
            self.get_logger().info("MOVIMIENTO DE CUERPO " + ("" if msg.data else "DES") + "BLOQUEADO")
            if not msg.data: # Reiniciamos los targets
                self.hri_body.target_x = self.body_x

        self.block_body_movement = msg.data

    def head_position_callback(self, msg):
        self.head_pan = msg.position[0]
        self.head_tilt = msg.position[1]
    
    def orientation_callback(self, msg):
        if self.block_body_rotation and not msg.force.data:
            self.get_logger().info("Se recibió una solicitud de rotación del cuerpo, pero está BLOQUEADO.")
        else:
            rot_z = float(msg.rot_z)

            if msg.mode.data == "absolute":
                self.hri_body.target_z = rot_z
            elif msg.mode.data == "relative":
                self.hri_body.target_z = self.body_z + rot_z

            self.get_logger().info("Solicitud de rotacion recibida: Z = " + str(self.hri_body.target_z) + " [" + str(msg.mode.data) + "]")
        
    def movement_callback(self, msg):
        if self.block_body_movement and not msg.force.data:
            self.get_logger().info("Se recibió una solicitud de movimiento del cuerpo, pero está BLOQUEADO.")
        else:
            mov_x = float(msg.mov_x)

            if msg.mode.data == "absolute":
                self.hri_body.target_x = mov_x
            elif msg.mode.data == "relative":
                self.hri_body.target_x = self.body_x + mov_x

            self.get_logger().info("Solicitud de movimiento recibida: X = " + str(self.hri_body.target_x) + " [" + str(msg.mode.data) + "]")

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        x, y, w, z = orientation.x, orientation.y, orientation.w, orientation.z

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        
        self.body_z = math.atan2(t3, t4)
        self.body_x = position.x

        self.last_motor_callback = time.time()


class HRIBody:

    def __init__(self):
        self.target_z = 0
        self.target_x = 0

        self.orientation_threshold = 0.1
        self.move_threshold = 0.1

        self.node = HRIBodyNode(self)

    def get_vel_value(self, diff, min=0.25, max=1):
        sign = -1.0 if diff < 0 else 1.0
        value = min if abs(diff) < min else max if abs(diff) > max else abs(diff)
        return sign * value

    def spin(self):
        while rclpy.ok():
            self.node.body_alive = time.time() - self.node.last_motor_callback < 1 # Si lleva mas de 1s sin publicar, murió
           
            diff_z = self.target_z - self.node.body_z
            diff_x = self.target_x - self.node.body_x

            over_threshold_z = abs(diff_z) > self.orientation_threshold
            over_threshold_x = abs(diff_x) > self.move_threshold

            moving_z = over_threshold_z and not self.node.block_body_rotation
            moving_x = over_threshold_x and not self.node.block_body_movement

            if moving_z or moving_x:
                z_vel = self.get_vel_value(diff_z) if moving_z else 0.0
                x_vel = self.get_vel_value(diff_x, max=0.5) if moving_x else 0.0

                self.node.cmd_vel.publish(Twist(linear=Vector3(x=x_vel,y=0.0,z=0.0),angular=Vector3(x=0.0,y=0.0,z=z_vel)))
                
                self.node.get_logger().info(("Rotando hacia " + str(self.target_z) + ". Estoy en Z: " + str(self.node.body_z) 
                                            if moving_z else "") + (" " if moving_z and moving_x else "") + 
                                            ("Moviendo hacia " + str(self.target_x) + ". Estoy en X: " + str(self.node.body_x) if moving_x else ""))

            if self.node.body_alive: # Si esta muerto no publica rotating (ni true ni false) y hri head detecta que esta muerto
                self.node.pub_rotating.publish(Bool(data=moving_z))
                self.node.pub_moving.publish(Bool(data=moving_x))

            rclpy.spin_once(self.node)


def main(args=None):
    rclpy.init(args=args)

    hri_body = HRIBody()
    hri_body.spin()

    rclpy.shutdown()
