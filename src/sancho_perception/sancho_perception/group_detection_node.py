#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion
from rclpy.duration import Duration
import numpy as np
from sklearn.cluster import DBSCAN

class GroupDetectionNode(Node):
    def __init__(self):
        super().__init__('group_detection')
        self.get_logger().info("Iniciando nodo de detección de grupos...")

        # Parámetros configurables
        self.declare_parameter('group_distance_threshold', 1.0)  # Distancia máxima entre personas para considerarlas en grupo (m)
        self.declare_parameter('min_group_duration', 3.0)          # Duración mínima (segundos) para confirmar grupo persistente
        self.declare_parameter('group_centroid_tolerance', 0.5)      # Tolerancia para considerar que el grupo es el mismo (m)
        self.declare_parameter('check_period', 0.5)                  # Periodo de verificación (s)
        self.declare_parameter('group_topic', '/detected_group')     # Tópico donde se publicará el grupo detectado

        self.group_distance_threshold = self.get_parameter('group_distance_threshold').value
        self.min_group_duration = self.get_parameter('min_group_duration').value
        self.group_centroid_tolerance = self.get_parameter('group_centroid_tolerance').value
        check_period = self.get_parameter('check_period').value
        self.group_topic = self.get_parameter('group_topic').value

        # Publicador para el grupo detectado (se usa PoseStamped para reutilizar la estructura de posición y orientación)
        self.group_pub = self.create_publisher(PoseStamped, self.group_topic, 10)
        # Subscripción a las poses de personas
        self.create_subscription(PoseArray, '/human_pose/persons_poses', self.poses_callback, 10)

        # Variables de estado para el seguimiento del grupo
        self.current_group_centroid = None  # np.array([x, y])
        self.group_start_time = None        # Tiempo en que se inició el seguimiento del grupo
        self.last_detection_time = None     # Última vez en que se detectó el grupo
        self.group_active = False           # Indica si el grupo ya fue publicado

        # Timer para comprobar la persistencia del grupo
        self.create_timer(check_period, self.timer_callback)

    def poses_callback(self, msg: PoseArray):
        poses = msg.poses
        if len(poses) < 2:
            # Si hay menos de 2 personas, se descarta la posibilidad de grupo
            self.reset_group_detection()
            return

        # Extraer posiciones 2D
        points = np.array([[pose.position.x, pose.position.y] for pose in poses])

        # Aplicar DBSCAN para identificar clusters
        clustering = DBSCAN(eps=self.group_distance_threshold, min_samples=2).fit(points)
        labels = clustering.labels_
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.discard(-1)  # Eliminar outliers

        if not unique_labels:
            self.reset_group_detection()
            return

        # Seleccionar el cluster con mayor cantidad de personas
        best_cluster = None
        best_count = 0
        for label in unique_labels:
            count = np.sum(labels == label)
            if count > best_count:
                best_count = count
                best_cluster = label

        # Calcular centroide del cluster seleccionado
        cluster_points = points[labels == best_cluster]
        centroid = np.mean(cluster_points, axis=0)

        now = self.get_clock().now()
        self.last_detection_time = now

        if self.current_group_centroid is None:
            # Iniciar seguimiento de un nuevo grupo
            self.current_group_centroid = centroid
            self.group_start_time = now
            self.group_active = False
            self.get_logger().info(f"Nuevo grupo detectado: centroide {centroid}. Iniciando seguimiento.")
        else:
            # Verificar si el grupo sigue siendo el mismo comparando centroide
            dist = np.linalg.norm(centroid - self.current_group_centroid)
            if dist < self.group_centroid_tolerance:
                # Actualizar centroide de forma progresiva
                self.current_group_centroid = 0.5 * (self.current_group_centroid + centroid)
            else:
                self.get_logger().info(f"Grupo distinto detectado (distancia {dist:.2f} m). Reiniciando seguimiento.")
                self.current_group_centroid = centroid
                self.group_start_time = now
                self.group_active = False

    def timer_callback(self):
        now = self.get_clock().now()
        # Si no se detecta grupo recientemente, reiniciar seguimiento
        if self.last_detection_time is None or (now - self.last_detection_time) > Duration(seconds=2.0):
            self.reset_group_detection()
            return

        if self.current_group_centroid is not None and self.group_start_time is not None:
            elapsed = (now - self.group_start_time).nanoseconds / 1e9
            if elapsed >= self.min_group_duration and not self.group_active:
                # Grupo persistente: publicar el centroide
                self.publish_group(self.current_group_centroid)
                self.group_active = True

    def publish_group(self, centroid):
        group_msg = PoseStamped()
        group_msg.header.stamp = self.get_clock().now().to_msg()
        group_msg.header.frame_id = "map"  # Ajusta según tu configuración
        group_msg.pose.position = Point(x=float(centroid[0]),
                                        y=float(centroid[1]),
                                        z=0.0)
        # Orientación fija (por ejemplo, mirando hacia adelante)
        group_msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.group_pub.publish(group_msg)
        self.get_logger().info(f"Grupo persistente detectado. Publicando grupo en {centroid}.")

    def reset_group_detection(self):
        if self.current_group_centroid is not None:
            self.get_logger().debug("Reiniciando seguimiento de grupo.")
        self.current_group_centroid = None
        self.group_start_time = None
        self.last_detection_time = None
        self.group_active = False

def main(args=None):
    rclpy.init(args=args)
    node = GroupDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado, cerrando nodo de detección...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
