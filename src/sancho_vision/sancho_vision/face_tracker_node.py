#!/usr/bin/env python3
"""ROS2 Humble FaceTracker mejorado:
- Control continuo a través de Timer en lugar de espera pasiva
- Filtro de media móvil exponencial (EMA) para suavizar detecciones
- Control proporcional (P) configurable sobre pan/tilt
- Manejo de timeouts y límites de ángulo
- Parametrizable por ROS2 params
"""
import math

import numpy as np
import rclpy
import tf_transformations
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sancho_msgs.msg import FaceArray
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Header


class FaceTracker(Node):
    def __init__(self):
        super().__init__("face_tracker")

        # --- Declaración de parámetros ---
        self.declare_parameter("face_topic", "/face_detections")
        self.declare_parameter("head_goal_topic", "/head_goal")
        self.declare_parameter("camera_info_topic", "/sancho_camera/camera_info")
        self.declare_parameter("camera_frame", "camera_frame")
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("ema_alpha", 0.2)
        self.declare_parameter("p_gain_pan", 0.8)
        self.declare_parameter("p_gain_tilt", 0.8)
        self.declare_parameter("timeout_no_detection", 1.0)
        self.declare_parameter("pan_limit_deg", 90.0)
        self.declare_parameter("tilt_limit_deg", 45.0)
        self.declare_parameter("pan_joint", "pan")
        self.declare_parameter("tilt_joint", "tilt")

        # --- Lectura inicial de parámetros ---
        self.face_topic = self.get_parameter("face_topic").value
        self.head_goal_topic = self.get_parameter("head_goal_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.control_rate = self.get_parameter("control_rate").value
        self.ema_alpha = self.get_parameter("ema_alpha").value
        self.p_gain_pan = self.get_parameter("p_gain_pan").value
        self.p_gain_tilt = self.get_parameter("p_gain_tilt").value
        self.timeout_no_detection = self.get_parameter("timeout_no_detection").value
        self.pan_limit_rad = math.radians(self.get_parameter("pan_limit_deg").value)
        self.tilt_limit_rad = math.radians(self.get_parameter("tilt_limit_deg").value)
        self.pan_joint = self.get_parameter("pan_joint").value
        self.tilt_joint = self.get_parameter("tilt_joint").value

        # --- Intrínsecos de la cámara ---
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_info_ready = False

        # Filtro EMA sobre el centro de la cara
        self.smoothed_u = None
        self.smoothed_v = None
        self.last_detection_time = None

        # Estados actuales de pan/tilt
        self.current_pan = 0.0
        self.current_tilt = 0.0

        # QoS para CameraInfo (sensor data)
        qos_cam = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, qos_cam
        )
        self.create_subscription(
            JointState, "/wxxms/joint_states", self.joint_states_callback, 10
        )
        # QoS para detecciones y comandos
        qos = QoSProfile(depth=10)
        self.create_subscription(FaceArray, self.face_topic, self.face_callback, qos)
        self.goal_pub = self.create_publisher(PoseStamped, self.head_goal_topic, qos)

        # Timer de control continuo
        timer_period = 1.0 / self.control_rate
        self.create_timer(timer_period, self.control_loop)

        self.get_logger().info("FaceTracker inicializado.")

    def joint_states_callback(self, msg: JointState):
        try:
            idx_pan = msg.name.index(self.pan_joint)
            idx_tilt = msg.name.index(self.tilt_joint)
        except ValueError:
            return
        self.current_pan = msg.position[idx_pan]
        self.current_tilt = msg.position[idx_tilt]

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_ready:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_matrix_inv = np.linalg.inv(
                np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])
            )
            self.camera_info_ready = True
            self.get_logger().info(
                f"CameraInfo OK: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}"
            )

    def face_callback(self, msg: FaceArray):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not msg.faces or not self.camera_info_ready:
            return

        # Selección de la cara más confiable
        best = max(msg.faces, key=lambda f: f.confidence)
        u, v = best.center.x, best.center.y

        # Inicializar EMA si es primera detección
        if self.smoothed_u is None:
            self.smoothed_u = u
            self.smoothed_v = v
        else:
            # EMA: nuevo = alpha * medición + (1-alpha) * anterior
            self.smoothed_u = (
                self.ema_alpha * u + (1 - self.ema_alpha) * self.smoothed_u
            )
            self.smoothed_v = (
                self.ema_alpha * v + (1 - self.ema_alpha) * self.smoothed_v
            )

        self.last_detection_time = now

    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        # Asegurarse de tener detección reciente
        if (
            self.smoothed_u is None
            or self.last_detection_time is None
            or now - self.last_detection_time > self.timeout_no_detection
        ):
            return

        # Errores de píxeles relativos al centro de la imagen
        err_x = self.smoothed_u - self.cx
        err_y = self.smoothed_v - self.cy

        # Conversión a ángulo con aproximación pinhole
        delta_pan = -math.atan2(err_x, self.fx)
        delta_tilt = math.atan2(err_y, self.fy)

        pan_cmd = self.current_pan + self.p_gain_pan * delta_pan
        tilt_cmd = self.current_tilt + self.p_gain_tilt * delta_tilt

        self.get_logger().info(
            f"err_px=({err_x:.1f},{err_y:.1f})  "
            f"∆rad=({delta_pan:+.3f},{delta_tilt:+.3f})  "
            f"cmd=({pan_cmd:+.3f},{tilt_cmd:+.3f})"
            f"pos=({self.current_pan:+.3f},{self.current_tilt:+.3f})"
        )

        # Límite de movimientos
        pan_cmd = np.clip(pan_cmd, -self.pan_limit_rad, self.pan_limit_rad)
        tilt_cmd = np.clip(tilt_cmd, -self.tilt_limit_rad, self.tilt_limit_rad)

        # Publicar meta de orientación en quaternion
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame

        q = tf_transformations.quaternion_from_euler(0.0, tilt_cmd, pan_cmd)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.goal_pub.publish(msg)
        self.get_logger().info(f"Publicado pan={pan_cmd:.3f}, tilt={tilt_cmd:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = FaceTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
