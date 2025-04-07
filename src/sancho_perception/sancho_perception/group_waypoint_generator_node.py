#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sancho_msgs.msg import GroupInfo

import numpy as np
import math
import tf_transformations  # Asegúrate de tener instalado: pip install tf-transformations


class GroupWaypointGeneratorNode(LifecycleNode):
    def __init__(self):
        super().__init__('group_waypoint_generator_lifecycle')
        self.get_logger().info("Inicializando nodo de generación de waypoints con Lifecycle...")

        # Declarar parámetros
        self.declare_parameter('group_topic', '/detected_group')
        self.declare_parameter('waypoint_goal_topic', '/group_waypoint')
        self.declare_parameter('robot_pose_topic', '/amcl_pose')
        self.declare_parameter('safety_margin', 0.5)
        self.declare_parameter('goal_update_threshold', 0.1)

        # Variables internas
        self.group_sub = None
        self.robot_pose_sub = None
        self.goal_pub = None

        self.robot_pose = None
        self.last_goal = None

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Configurando nodo...")

        self.group_topic = self.get_parameter('group_topic').value
        self.waypoint_goal_topic = self.get_parameter('waypoint_goal_topic').value
        self.robot_pose_topic = self.get_parameter('robot_pose_topic').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.goal_update_threshold = self.get_parameter('goal_update_threshold').value

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Activando nodo...")

        qos = QoSProfile(depth=10)

        self.group_sub = self.create_subscription(GroupInfo, self.group_topic, self.group_callback, qos)
        self.robot_pose_sub = self.create_subscription(PoseStamped, self.robot_pose_topic, self.robot_pose_callback, qos)
        self.goal_pub = self.create_publisher(PoseStamped, self.waypoint_goal_topic, qos)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Desactivando nodo...")

        self.destroy_subscription(self.group_sub)
        self.group_sub = None

        self.destroy_subscription(self.robot_pose_sub)
        self.robot_pose_sub = None

        self.destroy_publisher(self.goal_pub)
        self.goal_pub = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Limpiando recursos del nodo...")

        self.robot_pose = None
        self.last_goal = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("Apagando nodo...")

        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().error("¡Error en el nodo! Reiniciando estado interno...")
        self.robot_pose = None
        self.last_goal = None
        return TransitionCallbackReturn.SUCCESS

    def robot_pose_callback(self, msg: PoseStamped):
        self.robot_pose = msg

    def group_callback(self, msg: GroupInfo):
        try:
            centroid = np.array([msg.pose.position.x, msg.pose.position.y])
            radius = msg.radius
            self.get_logger().info(f"Recibido grupo: centroide {centroid.tolist()}, radio {radius:.2f}")
        except Exception as e:
            self.get_logger().error(f"Error al procesar GroupInfo: {e}")
            return
        
        if self.robot_pose is None:
            self.get_logger().warn("No se dispone de la pose actual del robot.")
            return

        try:
            robot_pos = np.array([self.robot_pose.pose.position.x, self.robot_pose.pose.position.y])
        except Exception as e:
            self.get_logger().error(f"Error al extraer la pose del robot: {e}")
            return

        # Vector desde el centroide al robot
        vec = robot_pos - centroid
        norm = np.linalg.norm(vec)
        if norm < 1e-3:
            self.get_logger().warn("La pose del robot coincide con el centro del grupo. Usando dirección por defecto.")
            vec = np.array([1.0, 0.0])
            norm = 1.0
        
        direction = vec / norm
        goal_point = centroid + (radius + self.safety_margin) * direction

        if self.last_goal is not None:
            diff = np.linalg.norm(goal_point - self.last_goal)
            if diff < self.goal_update_threshold:
                self.get_logger().info("Objetivo similar al anterior. No se actualiza.")
                return

        self.last_goal = goal_point

        # Orientación para mirar al grupo
        vec_to_group = centroid - goal_point
        yaw = math.atan2(vec_to_group[1], vec_to_group[0])
        q = self.yaw_to_quaternion(yaw)

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = msg.header.frame_id
        goal_msg.pose.position = Point(x=float(goal_point[0]), y=float(goal_point[1]), z=0.0)
        goal_msg.pose.orientation = q

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Publicado objetivo en {goal_point.tolist()} con yaw {yaw:.2f} rad.")

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q_arr = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=q_arr[0], y=q_arr[1], z=q_arr[2], w=q_arr[3])


