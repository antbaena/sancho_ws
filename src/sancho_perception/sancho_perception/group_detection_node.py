#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo de detección de grupos robusto con múltiples filtros temporales y espaciales.
"""
from typing import Optional, Tuple, Set, Deque, Dict
from collections import deque, defaultdict

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point, PoseArray, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import DBSCAN
from sklearn.metrics import silhouette_score

from sancho_msgs.msg import GroupInfo


def bounding_box_area(points: np.ndarray) -> float:
    """Área del rectángulo mínimo que engloba los puntos."""
    min_xy = points.min(axis=0)
    max_xy = points.max(axis=0)
    return float(np.prod(max_xy - min_xy))


class GroupDetectionNode(LifecycleNode):
    def __init__(self):
        super().__init__("group_detection_lifecycle")
        self.get_logger().info("Inicializando nodo de detección de grupos...")

        # Parámetros
        params = [
            ("group_distance_threshold", 1.0),
            ("min_group_duration", 3.0),
            ("group_centroid_tolerance", 0.5),
            ("check_period", 0.5),
            ("dbscan_min_samples", 2),
            ("detection_timeout", 2.0),
            ("message_timeout", 2.0),
            ("silhouette_threshold", 0.3),
            ("grace_period", 1.0),
            ("min_group_radius", 0.2),
            ("cluster_history_size", 10),
            ("cluster_history_required", 5),
        ]
        for name, default in params:
            self.declare_parameter(name, default)

        # Recursos internos
        self.group_pub = None
        self.marker_pub = None
        self.persons_sub = None
        self.timer = None

        # Estado de detección
        self.cluster_history: Deque[Tuple[np.ndarray, Set[int]]] = deque()
        self.track_history: Dict[int, Deque[Tuple[np.ndarray, rclpy.time.Time]]] = defaultdict(lambda: deque(maxlen=2))
        self.current_detected = False
        self.group_start_time: Optional[rclpy.time.Time] = None
        self.last_detection_time: Optional[rclpy.time.Time] = None
        self.last_msg_ts: Optional[rclpy.time.Time] = None
        self.absence_start: Optional[rclpy.time.Time] = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Configurando nodo...")
        # Leer parámetros
        self.group_distance_threshold = self.get_parameter("group_distance_threshold").value
        self.min_group_duration = self.get_parameter("min_group_duration").value
        self.group_centroid_tolerance = self.get_parameter("group_centroid_tolerance").value
        self.check_period = self.get_parameter("check_period").value
        self.dbscan_min_samples = self.get_parameter("dbscan_min_samples").value
        self.detection_timeout = self.get_parameter("detection_timeout").value
        self.message_timeout = self.get_parameter("message_timeout").value
        self.silhouette_threshold = self.get_parameter("silhouette_threshold").value
        self.grace_period = self.get_parameter("grace_period").value
        self.min_group_radius = self.get_parameter("min_group_radius").value
        self.cluster_history_size = self.get_parameter("cluster_history_size").value
        self.cluster_history_required = self.get_parameter("cluster_history_required").value

        self.group_pub = self.create_lifecycle_publisher(GroupInfo, "/detected_group", 10)
        self.marker_pub = self.create_lifecycle_publisher(MarkerArray, "/group_marker_array", 10)
        return super().on_configure(state)

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Activando nodo...")
        qos = QoSProfile(depth=10)
        self.persons_sub = self.create_subscription(PoseArray, "/human_pose/persons_poses",
                                                   self.poses_callback, qos)
        self.timer = self.create_timer(self.check_period, self.timer_callback)
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Desactivando nodo...")
        if self.timer: self.timer.cancel()
        if self.persons_sub: self.destroy_subscription(self.persons_sub)
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Limpiando recursos...")
        if self.timer: self.destroy_timer(self.timer)
        if self.group_pub: self.destroy_lifecycle_publisher(self.group_pub)
        if self.marker_pub: self.destroy_lifecycle_publisher(self.marker_pub)
        if self.persons_sub: self.destroy_subscription(self.persons_sub)
        return super().on_cleanup(state)

    def _check_message_gap(self, now: rclpy.time.Time) -> None:
        if self.last_msg_ts is None: return
        gap = (now - self.last_msg_ts).nanoseconds / 1e9
        if gap > self.message_timeout:
            self.get_logger().info(f"Gap de {gap:.2f}s sin mensajes, reset.")
            self.reset()

    def poses_callback(self, msg: PoseArray) -> None:
        now = rclpy.time.Time.from_msg(msg.header.stamp)
        self._check_message_gap(now)
        self.last_msg_ts = now

        poses = msg.poses
        if len(poses) < 2:
            self.reset()
            return

        points = np.array([[p.position.x, p.position.y] for p in poses])
        try:
            ids = list(msg.track_ids)
        except AttributeError:
            ids = list(range(len(poses)))

        valid = []
        for i, pid in enumerate(ids):
            self.track_history[pid].append((points[i], now))
            if len(self.track_history[pid]) == 2:
                valid.append(pid)
        speeds = []
        for pid in valid:
            p0, t0 = self.track_history[pid][0]
            p1, t1 = self.track_history[pid][1]
            dt = (t1 - t0).nanoseconds / 1e9
            if dt > 0:
                speeds.append(np.linalg.norm(p1 - p0) / dt)
        if speeds and np.std(speeds) > self.group_distance_threshold:
            self.get_logger().debug("Alta varianza de velocidades, descartando.")
            self.reset()
            return

        mask = np.isfinite(points).all(axis=1)
        points = points[mask]
        ids = [ids[i] for i, m in enumerate(mask) if m]

        area = bounding_box_area(points)
        density = len(points) / max(area, 1e-3)
        eps = self.group_distance_threshold * (1 + 0.5 * (density - 1))
        try:
            labels = DBSCAN(eps=eps, min_samples=self.dbscan_min_samples).fit_predict(points)
        except Exception as e:
            self.get_logger().error(f"DBSCAN error: {e}")
            self.reset()
            return

        unique = set(labels) - {-1}
        if not unique:
            if self.absence_start is None:
                self.absence_start = now
            elif (now - self.absence_start).nanoseconds/1e9 > self.grace_period:
                self.reset()
            return
        self.absence_start = None

        best_label = max(unique, key=lambda l: np.sum(labels == l))
        member_idx = np.where(labels == best_label)[0]
        member_ids = {ids[i] for i in member_idx}
        pts = points[labels == best_label]
        cen = pts.mean(axis=0)
        rad = float(np.linalg.norm(pts - cen, axis=1).max())

        if rad < self.min_group_radius:
            self.get_logger().debug("Radio de cluster muy pequeño.")
            self.reset()
            return

        # Calidad de cluster: solo si hay al menos 2 clusters válidos
        if len(points) >= 3:
            valid_clusters = set(labels) - {-1}
            if len(valid_clusters) > 1:
                try:
                    score = silhouette_score(points, labels)
                except ValueError as e:
                    self.get_logger().debug(f"Silhouette score error: {e}")
                else:
                    if score < self.silhouette_threshold:
                        self.get_logger().debug(f"Silhouette bajo ({score:.2f}).")
                        self.reset()
                        return

        # Historial de clusters
        self.cluster_history.append((cen, member_ids))
        if len(self.cluster_history) > self.cluster_history_size:
            self.cluster_history.popleft()
        stable = sum(
            np.linalg.norm(prev_cen - cen) < self.group_centroid_tolerance
            for prev_cen, _ in self.cluster_history
        )
        if stable < self.cluster_history_required:
            self.get_logger().debug(f"Cluster no suficientemente estable ({stable}).")
            return

        if not self.current_detected:
            self.group_start_time = now
            self.current_detected = True
            return
        elapsed = (now - self.group_start_time).nanoseconds/1e9
        if elapsed >= self.min_group_duration and not self.last_detection_time:
            self.publish_group(cen, rad, now)
            self.last_detection_time = now

    def timer_callback(self) -> None:
        now = self.get_clock().now()
        if self.last_detection_time and (now - self.last_detection_time) > Duration(seconds=self.detection_timeout):
            self.reset()

    def publish_group(self, centroid: np.ndarray, radius: float, timestamp: rclpy.time.Time) -> None:
        gm = GroupInfo()
        gm.header.stamp = timestamp.to_msg()
        gm.header.frame_id = "map"
        gm.pose.position = Point(x=float(centroid[0]),
                                 y=float(centroid[1]), z=0.0)
        gm.pose.orientation = Quaternion(w=1.0)
        gm.radius = radius
        self.group_pub.publish(gm)

        ma = MarkerArray()
        m = Marker()
        m.header = gm.header
        m.ns, m.id, m.type = "group", 0, Marker.SPHERE
        m.action = Marker.ADD
        m.pose = gm.pose
        m.scale.x = m.scale.y = m.scale.z = radius * 2
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.5
        m.lifetime = Duration(seconds=self.min_group_duration).to_msg()
        ma.markers.append(m)
        self.marker_pub.publish(ma)
        self.get_logger().info(f"Publicado grupo en {centroid.tolist()} radio {radius:.2f}")

    def reset(self) -> None:
        self.get_logger().debug("Reset estado de detección.")
        self.cluster_history.clear()
        self.current_detected = False
        self.group_start_time = None
        self.last_detection_time = None
        self.absence_start = None
        self.track_history.clear()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroupDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
