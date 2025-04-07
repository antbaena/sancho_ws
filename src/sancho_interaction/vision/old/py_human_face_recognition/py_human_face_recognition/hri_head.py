import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool

from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import OperatingModes

from human_face_recognition_msgs.msg import BodyOrientation, HeadOrientation
from human_face_recognition_msgs.srv import GetBool

from .hri_bridge import HRIBridge


class HRIHeadNode(Node):

    def __init__(self, hri_head):
        super().__init__("hri_head")

        self.pan = 0.0
        self.tilt = 0.0

        self.body_rotating = False
        self.body_alive = False
        self.last_body_callback = time.time()

        self.block_head_rotation = False

        self.sub_block_orientation = self.create_subscription(Bool, "/robot/head/block_orientation", self.block_orientation_callback, 1)
        self.sub_detection_data = self.create_subscription(JointState, "/wxxms/joint_states", self.head_position_callback, 10)
        self.sub_head_movement = self.create_subscription(HeadOrientation, "/head/movement", self.rotation_callback, 1)
        self.sub_body_rotating = self.create_subscription(Bool, "/robot/rotating", self.body_rotating_callback, 1)
        
        self.robot_orientation = self.create_publisher(BodyOrientation, "/robot/orientation", 1)
        self.pub_neck_joint = self.create_publisher(JointGroupCommand, "/wxxms/commands/joint_group", 1)

        self.is_body_orientation_blocked = self.create_client(GetBool, '/robot/body/is_orientation_blocked')
        self.serv_neck_op = self.create_client(OperatingModes, "/wxxms/set_operating_modes")
        while not self.serv_neck_op.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Head service not available, waiting...")
        
        self.hri_head = hri_head
        self.br = HRIBridge() 
        
        self.get_logger().info("Cabeza inicializada correctamente.")

    def is_orientation_blocked(self, request, response):
        response.value = Bool(data=self.block_head_rotation)
        return response

    def block_orientation_callback(self, msg):
        if self.block_head_rotation != msg.data:
            self.get_logger().info("ROTACION DE CABEZA " + ("" if msg.data else "DES") + "BLOQUEADA")
            if not msg.data: # Reiniciamos los targets
                self.hri_head.target_pan = None
                self.hri_head.target_tilt = None

        self.block_head_rotation = msg.data

    def body_rotating_callback(self, msg):
        self.body_rotating = msg.data
        
        self.last_body_callback = time.time()

    def head_position_callback(self, msg):
        self.pan = msg.position[0]
        self.tilt = msg.position[1]
    
    def rotation_callback(self, msg):
        self.get_logger().info("Force: " + str(msg.force.data))
        if self.block_head_rotation and not msg.force.data:
            self.get_logger().info("Se recibió una solicitud de rotación de la cabeza, pero está BLOQUEADA.")
        else:
            pan = float(msg.pan)
            tilt = float(msg.tilt)
            
            if msg.mode.data == "absolute":
                self.hri_head.target_pan = pan
                self.hri_head.target_tilt = tilt
            elif msg.mode.data == "relative":
                self.hri_head.target_pan = self.pan + pan
                self.hri_head.target_tilt =  self.tilt + tilt

            self.get_logger().info("Solicitud de rotacion recibida: (pan, tilt) = (" + str(round(self.hri_head.target_pan, 2))+ 
                ", " + str(round(self.hri_head.target_tilt, 2)) + ") [" + str(msg.mode.data) + "]")


class HRIHead:

    def __init__(self):
        self.target_pan = None
        self.target_tilt = None

        self.block_head_while_body_rotating = None
        
        self.node = HRIHeadNode(self)

        self.init_head()

    def spin(self):
        while rclpy.ok():
            self.node.body_alive = time.time() - self.node.last_body_callback < 1 # Si lleva mas de 1s sin publicar, murió

            is_body_orientation_blocked = self.get_body_rot_block_request()
            can_move_body = self.node.body_alive and not is_body_orientation_blocked

            if not can_move_body: # Si no puede mover el cuerpo desbloqueamos la cabeza por si murio rotando
                self.block_head_while_body_rotating = None

            if self.block_head_while_body_rotating is not None and time.time() - self.block_head_while_body_rotating > 0.2:
                if not self.node.body_rotating:
                    self.block_head_while_body_rotating = None  # Unlock the head and reset target values
                    
                    self.target_pan = None
                    self.target_tilt = None
            
            if self.block_head_while_body_rotating is None and self.target_pan is not None and self.target_tilt is not None:
                self.node.get_logger().info("Movement request: (" + str(round(self.target_pan, 2)) + ", " + str(round(self.target_tilt, 2)) + ")")
                
                if abs(self.target_pan) > 0.5 and can_move_body:
                    self.node.get_logger().info("Rotating body, large angle (Locking head)")
                    self.block_head_while_body_rotating = time.time()  # Lock the head during body rotation

                    self.node.robot_orientation.publish(BodyOrientation(mode=String(data="relative"), rot_z=float(self.target_pan)))
                    self.node.pub_neck_joint.publish(JointGroupCommand(name="turret", cmd=[0, self.node.tilt]))
                else:
                    cause = "body is not alive" if not self.node.body_alive else "body is blocked" if is_body_orientation_blocked else "small_angle"

                    self.node.get_logger().info("Moving only neck, " + cause)
                    self.node.pub_neck_joint.publish(JointGroupCommand(name="turret", cmd=[self.target_pan, self.target_tilt]))
                
                self.target_pan = None
                self.target_tilt = None

            rclpy.spin_once(self.node)

    def init_head(self):
        request = OperatingModes.Request()
        request.cmd_type = "group"
        request.name = "turret"
        request.mode = "position"
        request.profile_type = "time"
        request.profile_velocity = 1000
        request.profile_acceleration = 500

        future = self.node.serv_neck_op.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            self.node.get_logger().info("Service call successful. Response: %s" % future.result())
        else:
            self.node.get_logger().error("Service call failed")

        self.node.pub_neck_joint.publish(JointGroupCommand(name="turret", cmd=[0, 0]))
    
    def get_body_rot_block_request(self):
        result = False

        if self.node.is_body_orientation_blocked.service_is_ready():
            get_body_rot_block_request = GetBool.Request()

            future_get_body_rot_block = self.node.is_body_orientation_blocked.call_async(get_body_rot_block_request)
            rclpy.spin_until_future_complete(self.node, future_get_body_rot_block)
            result_get_body_rot_block = future_get_body_rot_block.result()
            
            result = result_get_body_rot_block.value.data
        else:
            self.node.body_alive = False
        
        return result


def main(args=None):
    rclpy.init(args=args)

    hri_head = HRIHead()
    hri_head.spin()

    rclpy.shutdown()
