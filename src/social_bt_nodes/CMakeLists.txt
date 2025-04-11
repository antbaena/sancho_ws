cmake_minimum_required(VERSION 3.8)
project(social_bt_nodes)

# Encuentra paquetes necesarios
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# Añade tu archivo fuente
add_library(social_bt_nodes SHARED
  src/is_human_in_path_condition.cpp
)

# Incluye los headers
target_include_directories(social_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Linkea contra las librerías necesarias
ament_target_dependencies(social_bt_nodes
  rclcpp
  nav2_behavior_tree
  nav2_costmap_2d
  geometry_msgs
  people_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp_v3
)

# Instala la librería
install(TARGETS social_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
