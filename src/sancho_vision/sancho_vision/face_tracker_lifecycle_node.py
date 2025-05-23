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
import numpy as np
from sensor_msgs.msg import JointState
import tf_transformations
from rclpy.executors import SingleThreadedExecutor


class FaceTrackerLifecycle(LifecycleNode):
    def __init__(self):
        super().__init__('face_tracker_lifecycle')

        # --- Declaración de parámetros (no leídos hasta on_configure) ---
        self.declare_parameter('face_topic', '/face_detections')
        self.declare_parameter('head_goal_topic', '/head_goal')
        self.declare_parameter('camera_info_topic', '/sancho_camera/camera_info')
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('control_rate', 10.0)
        self.declare_parameter('ema_alpha', 0.2)
        self.declare_parameter('p_gain_pan', 0.8)
        self.declare_parameter('p_gain_tilt', 0.8)
        self.declare_parameter('timeout_no_detection', 1.0)
        self.declare_parameter('pan_limit_deg', 90.0)
        self.declare_parameter('tilt_limit_deg', 45.0)
        self.declare_parameter('pan_joint', 'pan')
        self.declare_parameter('tilt_joint', 'tilt')

        # Valores que se inicializarán en configure
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

        self.get_logger().info('FaceTrackerLifecycle creado, esperando configuración.')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Lectura de parámetros
        self.face_topic         = self.get_parameter('face_topic').value
        self.head_goal_topic    = self.get_parameter('head_goal_topic').value
        self.camera_info_topic  = self.get_parameter('camera_info_topic').value
        self.camera_frame       = self.get_parameter('camera_frame').value
        self.control_rate       = self.get_parameter('control_rate').value
        self.ema_alpha          = self.get_parameter('ema_alpha').value
        self.p_gain_pan         = self.get_parameter('p_gain_pan').value
        self.p_gain_tilt        = self.get_parameter('p_gain_tilt').value
        self.timeout_no_detection = self.get_parameter('timeout_no_detection').value
        self.pan_limit_rad      = math.radians(self.get_parameter('pan_limit_deg').value)
        self.tilt_limit_rad     = math.radians(self.get_parameter('tilt_limit_deg').value)
        self.pan_joint          = self.get_parameter('pan_joint').value
        self.tilt_joint         = self.get_parameter('tilt_joint').value

        # QoS presets
        qos_default = QoSPresetProfiles.SYSTEM_DEFAULT.value

        
        # Publisher de Lifecycle
        self.goal_pub = self.create_lifecycle_publisher(
            PoseStamped,
            self.head_goal_topic,
            qos_default
        )

        self.get_logger().info('Configurado correctamente.')
        return super().on_configure(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        qos_sensor  = QoSPresetProfiles.SENSOR_DATA.value


        # Suscripciones y publisher (solo en configure)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_sensor
        )
        self.face_sub = self.create_subscription(
            FaceArray,
            self.face_topic,
            self.face_callback,
            qos_sensor
        )
        self.create_subscription(JointState,
                                 '/wxxms/joint_states',
                                 self.joint_states_callback,
                                 10)


         # Timer de control continuo
        timer_period = 1.0 / self.control_rate
        self.create_timer(timer_period, self.control_loop)

        self.get_logger().info('Activado: respondiendo a detecciones de caras.')
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self.control_timer)
        self.destroy_subscription(self.camera_info_sub)
        self.destroy_subscription(self.face_sub)
        self.destroy_subscription(self.joint_sub)

        self.goal_pub.on_deactivate()
        self.get_logger().info('Nodo desactivado.')
        return super().on_deactivate(state)


    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_ready:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.camera_info_ready = True

    def joint_states_callback(self, msg: JointState):
        try:
            idx_pan = msg.name.index(self.pan_joint)
            idx_tilt = msg.name.index(self.tilt_joint)
            self.current_pan = msg.position[idx_pan]
            self.current_tilt = msg.position[idx_tilt]
        except ValueError:
            pass

    def face_callback(self, msg: FaceArray):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        if not msg.faces or not self.camera_info_ready:
            return

        best = max(msg.faces, key=lambda f: f.confidence)
        u, v = best.center.x, best.center.y

        alpha = self.ema_alpha
        self.smoothed_u = u if self.smoothed_u is None else alpha*u + (1-alpha)*self.smoothed_u
        self.smoothed_v = v if self.smoothed_v is None else alpha*v + (1-alpha)*self.smoothed_v
        self.last_detection_time = now

    def control_loop(self):
        now = self.get_clock().now().seconds_nanoseconds()[0]
        if (self.smoothed_u is None or self.last_detection_time is None or
                now - self.last_detection_time > self.timeout_no_detection):
            return

        err_x = self.smoothed_u - self.cx
        err_y = self.smoothed_v - self.cy

        delta_pan = -math.atan2(err_x, self.fx)
        delta_tilt = math.atan2(err_y, self.fy)

        pan_cmd = np.clip(self.current_pan + self.p_gain_pan * delta_pan,
                          -self.pan_limit_rad, self.pan_limit_rad)
        tilt_cmd = np.clip(self.current_tilt + self.p_gain_tilt* delta_tilt,
                           -self.tilt_limit_rad, self.tilt_limit_rad)

        msg = PoseStamped()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.camera_frame)
        q = tf_transformations.quaternion_from_euler(0, tilt_cmd, pan_cmd)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q

        self.goal_pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = FaceTrackerLifecycle()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except Exception as e:
        node.get_logger().error(f'Exception in node: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
