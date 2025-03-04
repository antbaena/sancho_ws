#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
from rclpy.duration import Duration
import numpy as np
import math
from sklearn.cluster import DBSCAN

class GroupDetectionNavigatorNode(Node):
    def __init__(self):
        super().__init__('group_detection_navigator')
        self.get_logger().info("Iniciando nodo de detección de grupos y navegación...")

        # Parámetros configurables
        self.declare_parameter('group_distance_threshold', 1.0)  # Distancia máxima entre personas para considerarlas en grupo (m)
        self.declare_parameter('min_group_duration', 3.0)          # Duración mínima en segundos para confirmar grupo
        self.declare_parameter('group_centroid_tolerance', 0.5)      # Tolerancia en metros para considerar que el grupo sigue siendo el mismo
        self.declare_parameter('navigation_goal_topic', '/goal_pose')
        self.declare_parameter('check_period', 0.5)                  # Frecuencia (en s) del timer para comprobar persistencia

        self.group_distance_threshold = self.get_parameter('group_distance_threshold').value
        self.min_group_duration = self.get_parameter('min_group_duration').value
        self.group_centroid_tolerance = self.get_parameter('group_centroid_tolerance').value
        self.navigation_goal_topic = self.get_parameter('navigation_goal_topic').value
        check_period = self.get_parameter('check_period').value

        # Estado interno para seguimiento de grupo persistente
        self.current_group_centroid = None   # np.array([x, y])
        self.group_start_time = None         # rclpy.time.Time
        self.last_detection_time = None      # Último instante en el que se detectó el grupo
        self.goal_published = False           # Evitar publicar múltiples objetivos para el mismo grupo

        # Subscripción a las poses detectadas (PoseArray)
        self.create_subscription(PoseArray, '/human_pose/persons_poses', self.poses_callback, 10)

        # Publicador para el objetivo de navegación
        self.goal_pub = self.create_publisher(PoseStamped, self.navigation_goal_topic, 10)

        # Timer para verificar persistencia del grupo
        self.create_timer(check_period, self.timer_callback)

    def poses_callback(self, msg: PoseArray):
        # Extraer las posiciones 2D (x, y) de las poses detectadas
        poses = msg.poses
        if len(poses) < 2:
            # Si hay menos de 2 personas, no consideramos grupo
            self.reset_group_detection()
            return

        points = np.array([[pose.position.x, pose.position.y] for pose in poses])

        # Aplicar DBSCAN para detectar clusters (grupos)
        clustering = DBSCAN(eps=self.group_distance_threshold, min_samples=2).fit(points)
        labels = clustering.labels_
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.discard(-1)  # -1 son outliers

        if not unique_labels:
            # No se detecta ningún cluster válido
            self.reset_group_detection()
            return

        # Seleccionar el cluster con mayor número de elementos
        best_cluster = None
        best_count = 0
        for label in unique_labels:
            count = np.sum(labels == label)
            if count > best_count:
                best_count = count
                best_cluster = label

        # Calcular el centroide del cluster seleccionado
        cluster_points = points[labels == best_cluster]
        centroid = np.mean(cluster_points, axis=0)

        now = self.get_clock().now()
        self.last_detection_time = now

        # Si no se estaba siguiendo un grupo, inicializar el seguimiento
        if self.current_group_centroid is None:
            self.current_group_centroid = centroid
            self.group_start_time = now
            self.goal_published = False
            self.get_logger().info(f"Nuevo grupo detectado con centroide {centroid}. Iniciando seguimiento.")
        else:
            # Comparar el nuevo centroide con el que ya se estaba siguiendo
            dist = np.linalg.norm(centroid - self.current_group_centroid)
            if dist < self.group_centroid_tolerance:
                # Grupo consistente, actualizar el centroide (promediando)
                self.current_group_centroid = 0.5 * (self.current_group_centroid + centroid)
            else:
                # Se detectó un grupo distinto, reiniciar seguimiento
                self.get_logger().info(f"Grupo distinto detectado (distancia {dist:.2f} m). Reiniciando seguimiento.")
                self.current_group_centroid = centroid
                self.group_start_time = now
                self.goal_published = False

    def timer_callback(self):
        now = self.get_clock().now()
        # Si no se ha detectado grupo recientemente, reiniciar el estado
        if self.last_detection_time is None or (now - self.last_detection_time) > Duration(seconds=2.0):
            self.reset_group_detection()
            return

        if self.current_group_centroid is not None and self.group_start_time is not None:
            elapsed = (now - self.group_start_time).nanoseconds / 1e9
            if elapsed >= self.min_group_duration and not self.goal_published:
                # Grupo persistente detectado, publicar objetivo de navegación
                self.publish_navigation_goal(self.current_group_centroid)
                self.goal_published = True

    def publish_navigation_goal(self, centroid):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"  # Asegúrate de ajustar el frame según tu configuración
        goal_msg.pose.position = Point(x=float(centroid[0]),
                                       y=float(centroid[1]),
                                       z=0.0)
        # Orientación fija (por ejemplo, mirando hacia adelante)
        goal_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Grupo persistente detectado. Publicando objetivo de navegación hacia {centroid}.")

    def reset_group_detection(self):
        if self.current_group_centroid is not None:
            self.get_logger().debug("Reiniciando seguimiento de grupo.")
        self.current_group_centroid = None
        self.group_start_time = None
        self.last_detection_time = None
        self.goal_published = False

def main(args=None):
    rclpy.init(args=args)
    node = GroupDetectionNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado, cerrando nodo...")
    except Exception as e:
        node.get_logger().error(f"Error inesperado: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
