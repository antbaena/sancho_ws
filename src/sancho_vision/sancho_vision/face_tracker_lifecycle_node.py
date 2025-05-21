#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import State
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sancho_msgs.msg import FaceArray
from tf_transformations import quaternion_from_euler
from rclpy.qos import QoSPresetProfiles


class FaceTrackerLifecycle(LifecycleNode):
    def __init__(self):
        super().__init__('face_tracker_lifecycle')

        # --- Declaración de parámetros (no leídos hasta on_configure) ---
        self.declare_parameter('detection_duration', 0.0)
        self.declare_parameter('movement_threshold', 10.0)
        self.declare_parameter('face_topic', '/face_detections')
        self.declare_parameter('head_goal_topic', '/head_goal')
        self.declare_parameter('camera_info_topic', '/sancho_camera/camera_info')
        self.declare_parameter('camera_frame', 'camera_frame')

        # Valores que se inicializarán en configure
        self.fx = self.fy = self.cx = self.cy = None  
        self.camera_info_received = False
        self._tracking = False
        self._start_time = None
        self._last_center: Optional[Tuple[float, float]] = None

        self.get_logger().info('FaceTrackerLifecycle creado, esperando configuración.')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Lectura de parámetros
        self.detection_duration = self.get_parameter('detection_duration').value
        self.movement_threshold = self.get_parameter('movement_threshold').value
        self.face_topic         = self.get_parameter('face_topic').value
        self.head_goal_topic    = self.get_parameter('head_goal_topic').value
        self.camera_info_topic  = self.get_parameter('camera_info_topic').value
        self.camera_frame       = self.get_parameter('camera_frame').value

        # QoS presets
        qos_default = QoSPresetProfiles.SYSTEM_DEFAULT.value

        
        # Publisher de Lifecycle
        self.goal_pub = self.create_lifecycle_publisher(
            PoseStamped,
            self.head_goal_topic,
            qos_default
        )

        self.get_logger().info('Configurado: parámetros e interfaces creadas.')
        return super().on_configure(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        qos_sensor  = QoSPresetProfiles.SENSOR_DATA.value


        # Suscripciones y publisher (solo en configure)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._on_camera_info,
            qos_sensor
        )
        self.face_sub = self.create_subscription(
            FaceArray,
            self.face_topic,
            self._on_face_detections,
            qos_sensor
        )




        self.get_logger().info('Activado: respondiendo a detecciones de caras.')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Desactivamos el publisher
        self.destroy_subscription(self.camera_info_sub)
        self.destroy_subscription(self.face_sub)
        self.destroy_lifecycle_publisher(self.goal_pub)
        self.get_logger().info('Desactivado: dejando de publicar metas.')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # Destruye interfaces
        self.destroy_subscription(self.camera_info_sub)
        self.destroy_subscription(self.face_sub)
        self.destroy_lifecycle_publisher(self.goal_pub)
        self.get_logger().info('Limpiado: interfaces destruidas.')
        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutdown: FaceTrackerLifecycle detenido.')
        return TransitionCallbackReturn.SUCCESS

    def _on_camera_info(self, msg: CameraInfo) -> None:
        if self.camera_info_received:
            return
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        self.camera_info_received = True
        self.get_logger().info(
            f'Intrínsecos cámara → fx={self.fx:.2f}, fy={self.fy:.2f}, '
            f'cx={self.cx:.2f}, cy={self.cy:.2f}'
        )
        # Solo necesitamos el primero
        self.destroy_subscription(self.camera_info_sub)

    def _on_face_detections(self, msg: FaceArray) -> None:
        if not self.camera_info_received:
            self.get_logger().warn('Ignorando: sin CameraInfo.')
            return

        now = self.get_clock().now()
        if not msg.faces:
            self._reset()
            return

        face = max(msg.faces, key=lambda f: f.confidence)
        u, v = face.center.x, face.center.y

        if self._tracking and self._last_center:
            dx, dy = u - self._last_center[0], v - self._last_center[1]
            if math.hypot(dx, dy) > self.movement_threshold:
                self.get_logger().debug('Salto brusco, reiniciando.')
                self._tracking = False

        if not self._tracking:
            self._tracking = True
            self._start_time = now

        elapsed = (now - self._start_time).nanoseconds * 1e-9
        if elapsed >= self.detection_duration:
            pan, tilt = self._compute_angles(u, v)
            goal = PoseStamped()
            goal.header = Header(stamp=now.to_msg(), frame_id=self.camera_frame)
            q = quaternion_from_euler(0.0, tilt, pan)
            (goal.pose.orientation.x,
             goal.pose.orientation.y,
             goal.pose.orientation.z,
             goal.pose.orientation.w) = q
            self.goal_pub.publish(goal)
            self.get_logger().info(
                f'Publicado → pan: {pan:.3f}, tilt: {tilt:.3f}'
            )
            self._tracking = False

        self._last_center = (u, v)

    def _compute_angles(self, u: float, v: float) -> Tuple[float, float]:
        x_norm = (u - self.cx) / self.fx
        y_norm = (v - self.cy) / self.fy
        pan  = math.atan2(x_norm, 1.0)
        tilt = -math.atan2(y_norm, 1.0)
        return pan, tilt

    def _reset(self) -> None:
        self._tracking = False
        self._start_time = None
        self._last_center = None


def main(args=None):
    rclpy.init(args=args)
    node = FaceTrackerLifecycle()
    # Ahora el nodo está en estado UNCONFIGURED.
    # Puedes gestionar las transiciones con:
    #   ros2 lifecycle set /face_tracker configure
    #   ros2 lifecycle set /face_tracker activate
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
