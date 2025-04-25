#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.duration import Duration
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from sklearn.cluster import DBSCAN
from sancho_msgs.msg import GroupInfo

import numpy as np


class GroupDetectionNode(LifecycleNode):
    def __init__(self):
        super().__init__('group_detection_lifecycle')
        self.get_logger().info("Inicializando nodo de detección de grupos con Lifecycle...")

        # Declaración de parámetros
        self.declare_parameter('group_distance_threshold', 1.0)
        self.declare_parameter('min_group_duration', 3.0)
        self.declare_parameter('group_centroid_tolerance', 0.5)
        self.declare_parameter('check_period', 0.5)
        self.declare_parameter('group_topic', '/detected_group')
        self.declare_parameter('persons_topic', '/human_pose/persons_poses')
        self.declare_parameter('dbscan_min_samples', 2)
        self.declare_parameter('detection_timeout', 2.0)
        self.declare_parameter('message_timeout', 2.0)

        # Variables internas
        self.group_pub = None
        self.marker_pub = None
        self.persons_sub = None
        self.timer = None

        self.current_group_centroid = None
        self.current_group_radius = None
        self.group_start_time = None
        self.last_detection_time = None
        self.last_msg_timestamp = None
        self.group_active = False

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Configurando nodo...")

        self.group_distance_threshold = self.get_parameter('group_distance_threshold').value
        self.min_group_duration = self.get_parameter('min_group_duration').value
        self.group_centroid_tolerance = self.get_parameter('group_centroid_tolerance').value
        self.check_period = self.get_parameter('check_period').value
        self.group_topic = self.get_parameter('group_topic').value
        self.persons_topic = self.get_parameter('persons_topic').value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.message_timeout = self.get_parameter('message_timeout').value


        self.group_pub = self.create_lifecycle_publisher(GroupInfo, self.group_topic, 10)
        self.marker_pub = self.create_lifecycle_publisher(Marker, '/group_marker', 10)
        self.timer = self.create_timer(self.check_period, self.timer_callback)
        self.timer.cancel()

        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Activando nodo...")

        qos = QoSProfile(depth=10)

        self.persons_sub = self.create_subscription(PoseArray, self.persons_topic, self.poses_callback, qos)
        self.timer.reset()  # Reiniciar el timer para el chequeo de persistencia

        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Desactivando nodo...")

        self.timer.cancel()
        self.timer = None

        self.destroy_subscription(self.persons_sub)
        self.persons_sub = None

        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Limpiando recursos del nodo...")

        self.destroy_timer(self.timer)
        self.timer = None

        self.destroy_publisher(self.group_pub)
        self.group_pub = None

        self.destroy_publisher(self.marker_pub)
        self.marker_pub = None

        self.destroy_subscription(self.persons_sub)
        self.persons_sub = None

        self.reset_group_detection()

        return super().on_cleanup(state)

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Apagando nodo...")

        self.destroy_timer(self.timer)
        self.timer = None

        self.destroy_publisher(self.group_pub)
        self.group_pub = None

        self.destroy_publisher(self.marker_pub)
        self.marker_pub = None

        self.destroy_subscription(self.persons_sub)
        self.persons_sub = None
        return super().on_shutdown(state)

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error("Ha ocurrido un error. Nodo pasando a estado de error...")
        self.reset_group_detection()
        return super().on_error(state)

    def poses_callback(self, msg: PoseArray):
        current_timestamp = msg.header.stamp
        current_time = rclpy.time.Time.from_msg(current_timestamp)

        if self.last_msg_timestamp is not None:
            last_time = rclpy.time.Time.from_msg(self.last_msg_timestamp)
            gap = (current_time - last_time).nanoseconds / 1e9
            if gap > self.message_timeout:
                self.get_logger().info(
                    f"Gap de {gap:.2f} s entre mensajes. Reiniciando seguimiento."
                )
                self.reset_group_detection()

        self.last_msg_timestamp = current_timestamp

        poses = msg.poses
        if len(poses) < 2:
            self.reset_group_detection()
            return

        try:
            points = np.array([[pose.position.x, pose.position.y] for pose in poses])
            if np.any(np.isnan(points)) or np.any(np.isinf(points)):
                self.get_logger().warn("Datos inválidos en posiciones. Reiniciando seguimiento.")
                self.reset_group_detection()
                return

            clustering = DBSCAN(eps=self.group_distance_threshold, min_samples=self.dbscan_min_samples).fit(points)

        except Exception as e:
            self.get_logger().error(f"Error en DBSCAN: {e}")
            self.reset_group_detection()
            return

        labels = clustering.labels_
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.discard(-1)

        if not unique_labels:
            self.reset_group_detection()
            return

        best_cluster = max(unique_labels, key=lambda l: np.sum(labels == l))
        cluster_points = points[labels == best_cluster]
        centroid = np.mean(cluster_points, axis=0)
        distances = np.linalg.norm(cluster_points - centroid, axis=1)
        radius = float(np.max(distances))

        if self.current_group_centroid is None:
            self.current_group_centroid = centroid
            self.current_group_radius = radius
            self.group_start_time = current_time
            self.group_active = False
            self.get_logger().info(f"Nuevo grupo detectado: centroide {centroid}, radio {radius:.2f}")
        else:
            dist = np.linalg.norm(centroid - self.current_group_centroid)
            if dist < self.group_centroid_tolerance:
                self.current_group_centroid = 0.5 * (self.current_group_centroid + centroid)
                self.current_group_radius = 0.5 * (self.current_group_radius + radius)
            else:
                self.get_logger().info(f"Grupo diferente detectado (distancia {dist:.2f} m). Reiniciando seguimiento.")
                self.current_group_centroid = centroid
                self.current_group_radius = radius
                self.group_start_time = current_time
                self.group_active = False

        elapsed = (current_time - self.group_start_time).nanoseconds / 1e9
        if elapsed >= self.min_group_duration and not self.group_active:
            self.publish_group(self.current_group_centroid, self.current_group_radius, current_time)
            self.group_active = True

    def timer_callback(self):
        now = self.get_clock().now()
        if self.last_detection_time is None or (now - self.last_detection_time) > Duration(seconds=self.detection_timeout):
            self.reset_group_detection()

    def publish_group(self, centroid, radius, timestamp):
        if self.group_pub is None:
            return

        group_msg = GroupInfo()
        group_msg.header.stamp = timestamp.to_msg()
        group_msg.header.frame_id = "map"
        group_msg.pose.position = Point(x=float(centroid[0]),
                                        y=float(centroid[1]),
                                        z=0.0)
        group_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        group_msg.radius = radius

        self.group_pub.publish(group_msg)

        marker = Marker()
        marker.header.stamp = timestamp.to_msg()
        marker.header.frame_id = "map"
        marker.ns = "group"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = group_msg.pose.position
        marker.pose.orientation = group_msg.pose.orientation
        marker.scale.x = marker.scale.y = marker.scale.z = radius * 2
        marker.color.a = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = Duration(seconds=2.0).to_msg()

        self.marker_pub.publish(marker)

        self.get_logger().info(
            f"Grupo persistente detectado. Publicando centroide {centroid.tolist()} y radio {radius:.2f}"
        )

    def reset_group_detection(self):
        if self.current_group_centroid is not None:
            self.get_logger().debug("Reiniciando seguimiento de grupo.")
        self.current_group_centroid = None
        self.current_group_radius = None
        self.group_start_time = None
        self.last_detection_time = None
        self.group_active = False

def main(args=None):
    rclpy.init(args=args)

    # Creamos el nodo de ciclo de vida
    node = GroupDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Limpieza
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
