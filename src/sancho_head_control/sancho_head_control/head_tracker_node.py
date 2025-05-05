#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sancho_msgs.msg import FaceArray
import tf_transformations


class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')

        # --- Parámetros declarados ---
        self.declare_parameter('detection_duration', 0.0)
        self.declare_parameter('face_topic', '/face_detections')
        self.declare_parameter('head_goal_topic', '/head_goal')
        self.declare_parameter('camera_info_topic', '/sancho_camera/camera_info')
        self.declare_parameter('camera_frame', 'camera_frame')

        # --- Lectura de parámetros ---
        self.detection_duration = self.get_parameter('detection_duration').value
        self.face_topic        = self.get_parameter('face_topic').value
        self.head_goal_topic   = self.get_parameter('head_goal_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame      = self.get_parameter('camera_frame').value

        # --- Variables de cámara ---
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_ready = False

        # QoS para camera_info: datos de sensor (best_effort, volatile)
        qos_cam = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE)

        # Suscripción a CameraInfo
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_cam)

        # QoS para detecciones y comandos (reliable, depth suficiente)
        qos = QoSProfile(depth=10)

        # Suscriptor de detecciones de caras
        self.face_sub = self.create_subscription(
            FaceArray,
            self.face_topic,
            self.face_callback,
            qos)

        # Publicador de metas de cabeza
        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.head_goal_topic,
            qos)

        # Estado interno de tracking
        self.tracking = False
        self.detection_start_time = None
        self.last_center = None
        self.goal_sent = False

        self.get_logger().info('FaceTracker inicializado.')

    def camera_info_callback(self, msg: CameraInfo):
        # Extrae intrínsecos solo una vez
        if not self.camera_info_ready:
            # msg.K es lista de 9 floats: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_ready = True
            self.get_logger().info(
                f'CameraInfo recibida: fx={self.fx:.1f}, fy={self.fy:.1f}, '
                f'cx={self.cx:.1f}, cy={self.cy:.1f}')
            # Si no necesitas más datos de camera_info, puedes desuscribirte:
            # self.destroy_subscription(self.camera_info_sub)
    
    def face_callback(self, msg: FaceArray):
        now = self.get_clock().now()

        # Si aún no tenemos parámetros de cámara, no procesamos
        if not self.camera_info_ready:
            self.get_logger().warn('Esperando CameraInfo…')
            return

        # Si no hay detecciones, reset
        if not msg.faces:
            self._reset_tracking()
            return

        # Elige la detección de mayor confianza
        best = max(msg.faces, key=lambda f: f.confidence)
        u, v = best.center.x, best.center.y
        self.get_logger().info(
            f'Detección en píxel ({u:.1f}, {v:.1f}), conf={best.confidence:.2f}')

        # Gestionar estado de estabilidad
        if self.tracking:
            dx = u - self.last_center[0]
            dy = v - self.last_center[1]
            if math.hypot(dx, dy) > 10.0:
                # Movimiento brusco: reinicia
                self.tracking = False
        if not self.tracking:
            # Nuevo tracking estable
            self.tracking = True
            self.detection_start_time = now
            self.last_center = (u, v)
            self.goal_sent = False

        # Si lleva tiempo estable suficiente y no hemos enviado meta
        if self.tracking and not self.goal_sent:
            elapsed = (now - self.detection_start_time).nanoseconds * 1e-9
            if elapsed >= self.detection_duration:
                # Proyección pinhole:
                x_norm = (u - self.cx) / self.fx
                y_norm = (v - self.cy) / self.fy
                pan  = math.atan(x_norm)
                tilt = -math.atan(y_norm)

                # Construye PoseStamped
                goal = PoseStamped()
                goal.header = Header()
                goal.header.stamp = now.to_msg()
                goal.header.frame_id = self.camera_frame

                q = tf_transformations.quaternion_from_euler(0.0, tilt, pan)
                goal.pose.orientation.x = q[0]
                goal.pose.orientation.y = q[1]
                goal.pose.orientation.z = q[2]
                goal.pose.orientation.w = q[3]

                # Publica
                self.goal_pub.publish(goal)
                self.get_logger().info(
                    f'Publicado head goal — pan: {pan:.3f} rad, tilt: {tilt:.3f} rad')
                self.goal_sent = True

    def _reset_tracking(self):
        self.tracking = False
        self.detection_start_time = None
        self.last_center = None
        self.goal_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = FaceTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
