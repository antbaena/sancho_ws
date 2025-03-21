#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import numpy as np
import math
import tf_transformations  # Asegúrate de tener instalado tf_transformations
from sancho_msgs.msg import GroupInfo  # Mensaje con header, pose (centroide) y float32 radius
from std_msgs.msg import Bool  # Para mensajes de interacción

class GroupNavigationNode(Node):
    def __init__(self):
        super().__init__('group_navigation')
        self.get_logger().info("Iniciando nodo de navegación a grupos...")

        # Parámetros configurables
        self.declare_parameter('group_topic', '/detected_group')         # Tópico de grupos detectados
        self.declare_parameter('navigation_goal_topic', '/goal_pose')      # Tópico para publicar el objetivo de navegación
        self.declare_parameter('robot_pose_topic', '/amcl_pose')           # Tópico donde se publica la pose actual del robot
        self.declare_parameter('safety_margin', 0.5)                       # Margen extra (m) para el objetivo fuera del grupo
        self.declare_parameter('goal_update_threshold', 0.1)               # Distancia mínima para actualizar el objetivo
        self.declare_parameter('arrival_threshold', 0.3)                   # Umbral de llegada al objetivo

        self.group_topic = self.get_parameter('group_topic').value
        self.navigation_goal_topic = self.get_parameter('navigation_goal_topic').value
        self.robot_pose_topic = self.get_parameter('robot_pose_topic').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.goal_update_threshold = self.get_parameter('goal_update_threshold').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value

        # Subscripción a grupos detectados
        self.create_subscription(GroupInfo, self.group_topic, self.group_callback, 10)
        # Subscripción a la pose actual del robot
        self.create_subscription(PoseStamped, self.robot_pose_topic, self.robot_pose_callback, 10)
        # Subscripción para recibir el estado de interacción
        self.create_subscription(Bool, '/interaction_active', self.interaction_callback, 10)
        # Publicador para el objetivo de navegación
        self.goal_pub = self.create_publisher(PoseStamped, self.navigation_goal_topic, 10)
        # Publicador para indicar que se llegó al grupo y se puede iniciar la interacción
        self.interaction_enable_pub = self.create_publisher(Bool, '/interaction_enabled', 10)


        # Variables para almacenar el estado
        self.robot_pose = None      # Pose actual del robot (PoseStamped)
        self.last_goal = None       # Último objetivo publicado (np.array [x, y])
        self.interaction_active = False  # Flag que bloquea la navegación durante la interacción

        # Timer para verificar llegada al objetivo
        self.create_timer(0.5, self.check_goal_reached)

    def robot_pose_callback(self, msg: PoseStamped):
        # Actualizar la pose actual del robot
        self.robot_pose = msg
    
    def interaction_callback(self, msg: Bool):
        # Actualizar el estado de interacción basado en el mensaje recibido
        self.interaction_active = msg.data
        if self.interaction_active:
            self.get_logger().info("Interacción en curso. Bloqueo de actualización de navegación.")
        else:
            self.get_logger().info("Interacción finalizada. Se reanuda la navegación.")


    def group_callback(self, msg: GroupInfo):
        if self.interaction_active:
            self.get_logger().info("Modo interacción activo. No se actualiza el objetivo de navegación.")
            return
        try:
            # Extraer el centroide y radio del grupo
            centroid = np.array([msg.pose.position.x, msg.pose.position.y])
            radius = msg.radius
            self.get_logger().info(f"Recibido grupo: centroide {centroid}, radio {radius:.2f}")
        except Exception as e:
            self.get_logger().error(f"Error al procesar mensaje GroupInfo: {e}")
            return
        
        if self.robot_pose is None:
            self.get_logger().warn("No se dispone de la pose actual del robot. No se puede calcular el objetivo de navegación.")
            return

        try:
            # Extraer la posición actual del robot
            robot_pos = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y])
        except Exception as e:
            self.get_logger().error(f"Error al extraer la pose del robot: {e}")
            return

        # Calcular el vector desde el centro del grupo hacia la posición del robot
        vec = robot_pos - centroid
        norm = np.linalg.norm(vec)
        if norm < 1e-3:
            # Si el robot se encuentra muy cerca del centro del grupo, definir una dirección predeterminada (ej. eje X positivo)
            self.get_logger().warn("La pose del robot coincide con el centro del grupo. Usando dirección por defecto.")
            vec = np.array([1.0, 0.0])
            norm = 1.0
        
        direction = vec / norm
        # Calcular el punto objetivo en el perímetro del grupo: centroide + (radio + safety_margin) * dirección
        goal_point = centroid + (radius + self.safety_margin) * direction

        # Publicar el objetivo solo si difiere significativamente del último publicado
        if self.last_goal is not None:
            diff = np.linalg.norm(goal_point - self.last_goal)
            if diff < self.goal_update_threshold:
                self.get_logger().info("Objetivo similar al último publicado. No se actualiza.")
                return

        self.last_goal = goal_point

        # Calcular la orientación para que el robot mire hacia el grupo (es decir, apunte al centroide)
        vec_to_group = centroid - goal_point
        yaw = math.atan2(vec_to_group[1], vec_to_group[0])
        q = self.yaw_to_quaternion(yaw)

        # Crear y completar el mensaje de objetivo de navegación
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        # Usar el mismo frame del mensaje de grupo (por ejemplo, "map")
        goal_msg.header.frame_id = msg.header.frame_id
        goal_msg.pose.position = Point(x=float(goal_point[0]),
                                       y=float(goal_point[1]),
                                       z=0.0)
        goal_msg.pose.orientation = q

        # Publicar el objetivo
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Publicando objetivo de navegación: {goal_point} con orientación yaw {yaw:.2f}")

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        # Convierte un ángulo yaw (radianes) a un quaternion
        q_arr = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=q_arr[0], y=q_arr[1], z=q_arr[2], w=q_arr[3])
    
    def check_goal_reached(self):
            """
            Verifica periódicamente si el robot ha alcanzado el objetivo de navegación.
            Si la distancia entre la pose actual y el último objetivo es menor que un umbral,
            se notifica al módulo de interacción y se activa el modo interacción.
            """
            if self.interaction_active:
                # Ya está en modo interacción, no se chequea la llegada.
                return

            if self.robot_pose is None or self.last_goal is None:
                return

            # Calcular la distancia entre la pose actual y el último objetivo
            current_pos = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y])
            distance = np.linalg.norm(current_pos - self.last_goal)

            if distance < self.arrival_threshold:
                self.get_logger().info("Objetivo alcanzado. Habilitando interacción.")
                # Publicar mensaje para habilitar la interacción
                self.interaction_enable_pub.publish(Bool(data=True))
                # Activar el flag para bloquear nuevas actualizaciones de navegación
                self.interaction_active = True

def main(args=None):
    rclpy.init(args=args)
    node = GroupNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado, cerrando nodo de navegación...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
