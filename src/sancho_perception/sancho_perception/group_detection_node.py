#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped, Point, Quaternion
from rclpy.duration import Duration
import numpy as np
from sklearn.cluster import DBSCAN
from sancho_msgs.msg import GroupInfo

class GroupDetectionNode(Node):
    """
    GroupDetectionNode is a ROS 2 node that detects groups of people based on clustering their 2D positions.
    It subscribes to a topic providing a PoseArray of human poses, applies the DBSCAN algorithm to identify clusters,
    and tracks a persistent group based on the stability of its centroid and radius over time. When a group is deemed
    persistent (after a configurable minimum duration), the node publishes the group's centroid and radius as a GroupInfo message.
    Attributes:
        group_distance_threshold (float): Maximum distance allowed between persons to consider them part of the same group.
        min_group_duration (float): Minimum time in seconds for a group to be considered persistent.
        group_centroid_tolerance (float): Maximum tolerance (in meters) allowed for the centroid shift to continue tracking the same group.
        group_topic (str): Topic name where the detected group’s information is published.
        persons_topic (str): Topic name from which persons' poses are received.
        dbscan_min_samples (int): Minimum number of samples required by DBSCAN to form a cluster.
        detection_timeout (float): Timeout in seconds after which the detection process is reset if no detections occur.
        current_group_centroid (np.array or None): The current centroid of the tracked group.
        current_group_radius (float or None): The current computed radius of the tracked group.
        group_start_time (Time or None): Timestamp marking the beginning of the current group tracking.
        last_detection_time (Time or None): Timestamp of the most recent group detection.
        group_active (bool): Flag indicating if the group has been published as persistent.
    Methods:
        __init__():
            Initializes the node, declares and retrieves parameters, sets up the publisher and subscriber,
            and initializes variables for group tracking including a periodic timer for persistence checks.
        poses_callback(msg: PoseArray):
            Processes incoming PoseArray messages, extracts 2D positions, applies DBSCAN clustering to detect clusters,
            updates or resets the current group tracking based on the detected clusters, and logs information about the process.
        timer_callback():
            Invoked periodically based on a configured check period. It checks if a group is still being detected by comparing
            the current time with the last detection time. If the group is persistent for at least the minimum duration,
            it publishes the group's details; otherwise, it resets the group tracking.
        publish_group(centroid, radius):
            Creates and publishes a GroupInfo message using the provided centroid and radius of the persistent group.
            It stamps the message with the current time and assigns a static orientation and frame ("map").
        reset_group_detection():
            Resets the internal state variables used for tracking a group, including the centroid, radius,
            start time, last detection time, and active status, effectively clearing any ongoing group detection.
    """
    def __init__(self):
        super().__init__('group_detection')
        self.get_logger().info("Iniciando nodo de detección de grupos...")

        # Parámetros configurables
        self.declare_parameter('group_distance_threshold', 1.0)  # Distancia máxima entre personas para considerarlas en grupo (m)
        self.declare_parameter('min_group_duration', 3.0)          # Duración mínima (segundos) para confirmar grupo persistente
        self.declare_parameter('group_centroid_tolerance', 0.5)      # Tolerancia para considerar que el grupo es el mismo (m)
        self.declare_parameter('check_period', 0.5)                  # Periodo de verificación (s)
        self.declare_parameter('group_topic', '/detected_group')     # Tópico donde se publicará el grupo detectado
        self.declare_parameter('persons_topic', '/human_pose/persons_poses')
        self.declare_parameter('dbscan_min_samples', 2)
        self.declare_parameter('detection_timeout', 2.0)
        self.declare_parameter('message_timeout', 2.0)


        self.group_distance_threshold = self.get_parameter('group_distance_threshold').value
        self.min_group_duration = self.get_parameter('min_group_duration').value
        self.group_centroid_tolerance = self.get_parameter('group_centroid_tolerance').value
        check_period = self.get_parameter('check_period').value
        self.group_topic = self.get_parameter('group_topic').value
        persons_topic = self.get_parameter('persons_topic').value
        self.dbscan_min_samples = self.get_parameter('dbscan_min_samples').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.message_timeout = self.get_parameter('message_timeout').value


        # Publicador para el grupo detectado (se usa PoseStamped para reutilizar la estructura de posición y orientación)
        self.group_pub = self.create_publisher(PoseStamped, self.group_topic, 10)
        # Subscripción a las poses de personas
        self.create_subscription(PoseArray, persons_topic , self.poses_callback, 10)

        # Variables de estado para el seguimiento del grupo
        self.current_group_centroid = None  # np.array([x, y])
        self.current_group_radius = None      # Radio del grupo (float)
        self.group_start_time = None        # Tiempo en que se inició el seguimiento del grupo
        self.last_detection_time = None     # Última vez en que se detectó el grupo
        self.group_active = False           # Indica si el grupo ya fue publicado

        # Timer para comprobar la persistencia del grupo
        self.create_timer(check_period, self.timer_callback)

    def poses_callback(self, msg: PoseArray):
        
        # Extraer el timestamp del mensaje (tiempo real de la imagen)
        current_timestamp = msg.header.stamp  # Tipo builtin_msg.Time
        current_time = rclpy.time.Time.from_msg(current_timestamp)

        # Verificar si existe un gap demasiado grande entre mensajes
        if self.last_msg_timestamp is not None:
            last_time = rclpy.time.Time.from_msg(self.last_msg_timestamp)
            gap = (current_time - last_time).nanoseconds / 1e9
            if gap > self.message_timeout:
                self.get_logger().info(
                    f"Gap de {gap:.2f} s entre mensajes (máximo permitido {self.message_timeout} s). Reiniciando seguimiento."
                )
                self.reset_group_detection()

        # Actualizar el timestamp del último mensaje recibido
        self.last_msg_timestamp = current_timestamp


        poses = msg.poses
        if len(poses) < 2:
            # Si hay menos de 2 personas, se descarta la posibilidad de grupo
            self.reset_group_detection()
            return

        try:
            # Extraer posiciones 2D de las poses
            points = np.array([[pose.position.x, pose.position.y] for pose in poses])
        except Exception as e:
            self.get_logger().error(f"Error al extraer posiciones: {e}")
            return
        try:
            # Aplicar DBSCAN para identificar clusters
            clustering = DBSCAN(eps=self.group_distance_threshold, min_samples=self.dbscan_min_samples).fit(points)
        except Exception as e:
            self.get_logger().error(f"Error en DBSCAN: {e}")
            self.reset_group_detection()
            return
        
        labels = clustering.labels_
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.discard(-1)  # Eliminar outliers

        if not unique_labels:
            self.reset_group_detection()
            return

        # Seleccionar el cluster con mayor cantidad de personas
        best_cluster = max(unique_labels, key=lambda l: np.sum(labels == l))
        # Calcular centroide del cluster seleccionado
        cluster_points = points[labels == best_cluster]
        centroid = np.mean(cluster_points, axis=0)

        # Calcular el radio como la distancia máxima desde el centroide a los puntos del cluster
        distances = np.linalg.norm(cluster_points - centroid, axis=1)
        radius = float(np.max(distances))

        timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.last_detection_time = timestamp

        if self.current_group_centroid is None:
            # Iniciar seguimiento de un nuevo grupo
            self.current_group_centroid = centroid
            self.current_group_radius = radius
            self.group_start_time = timestamp
            self.group_active = False
            self.get_logger().info(f"Nuevo grupo detectado: centroide {centroid}, radio {radius:.2f}. Iniciando seguimiento.")
        else:
            # Verificar si el grupo sigue siendo el mismo comparando centroide
            dist = np.linalg.norm(centroid - self.current_group_centroid)
            if dist < self.group_centroid_tolerance:
                # Actualizar centroide de forma progresiva
                self.current_group_centroid = 0.5 * (self.current_group_centroid + centroid)
                self.current_group_radius = 0.5 * (self.current_group_radius + radius)

            else:
                self.get_logger().info(f"Grupo distinto detectado (distancia {dist:.2f} m). Reiniciando seguimiento.")
                self.current_group_centroid = centroid
                self.current_group_radius = radius
                self.group_start_time = timestamp
                self.group_active = False

        # Calcular el tiempo transcurrido usando el timestamp del mensaje actual y el de inicio
        elapsed = (current_time - self.group_start_time).nanoseconds / 1e9
        if elapsed >= self.min_group_duration and not self.group_active:
            self.publish_group(self.current_group_centroid, self.current_group_radius, current_time)
            self.group_active = True
        

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
                self.publish_group(self.current_group_centroid, self.current_group_radius)
                self.group_active = True

    def publish_group(self, centroid, radius, timestamp):
        group_msg = GroupInfo()
        group_msg.header.stamp = timestamp.to_msg()
        group_msg.header.frame_id = "map"  # Ajusta según tu configuración
        group_msg.group_pose.position = Point(x=float(centroid[0]),
                                        y=float(centroid[1]),
                                        z=0.0)
        # Orientación fija (por ejemplo, mirando hacia adelante)
        group_msg.group_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        group_msg.radius = radius
        self.group_pub.publish(group_msg)
        self.get_logger().info(
            f"Grupo persistente detectado. Publicando centroide {centroid} y radio {radius:.2f} con timestamp de imagen."
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
