#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from sancho_msgs.msg import GroupInfo
class GroupNavigationNode(Node):
    def __init__(self):
        super().__init__('group_navigation')
        self.get_logger().info("Iniciando nodo de navegación a grupos...")

        # Parámetros configurables
        self.declare_parameter('group_topic', '/detected_group')         # Tópico de grupos detectados
        self.declare_parameter('navigation_goal_topic', '/goal_pose')      # Tópico para publicar el objetivo de navegación

        self.group_topic = self.get_parameter('group_topic').value
        self.navigation_goal_topic = self.get_parameter('navigation_goal_topic').value

        # Subscripción al tópico de grupos detectados
        self.create_subscription(GroupInfo, self.group_topic, self.group_callback, 10)
        # Publicador para el objetivo de navegación
        self.goal_pub = self.create_publisher(PoseStamped, self.navigation_goal_topic, 10)

def group_callback(self, msg):
    # Suponiendo que msg contiene:
    #   - msg.group_pose: PoseStamped con el centroide del grupo
    #   - msg.radius: radio del grupo
    centroid = np.array([msg.group_pose.pose.position.x, msg.group_pose.pose.position.y])
    group_radius = msg.radius
    
    # Supongamos que tienes una función o suscripción para obtener la posición actual del robot
    robot_pose = self.get_robot_position()  # [x, y]
    
    # Calcular la dirección de aproximación (del centroide hacia el robot)
    direction = robot_pose - centroid
    norm = np.linalg.norm(direction)
    if norm > 0:
        unit_direction = direction / norm
    else:
        unit_direction = np.array([1.0, 0.0])  # dirección por defecto si no se puede calcular
    
    # Definir un margen de seguridad para respetar el espacio personal
    safety_margin = 0.5  # en metros, ajustar según sea necesario
    
    # Calcular el punto objetivo: sobre el borde exterior del grupo, alejándose del centro
    goal_point = centroid + (group_radius + safety_margin) * unit_direction
    
    # Publicar el objetivo de navegación
    nav_goal = PoseStamped()
    nav_goal.header.stamp = self.get_clock().now().to_msg()
    nav_goal.header.frame_id = msg.group_pose.header.frame_id  # por ejemplo, "map"
    nav_goal.pose.position.x = float(goal_point[0])
    nav_goal.pose.position.y = float(goal_point[1])
    nav_goal.pose.position.z = 0.0
    # Asigna una orientación fija o calcula la deseada según la situación
    nav_goal.pose.orientation.w = 1.0
    
    self.goal_pub.publish(nav_goal)
    self.get_logger().info(f"Objetivo de navegación modificado: {goal_point} (radio: {group_radius:.2f} m)")

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
