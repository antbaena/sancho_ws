cmake_minimum_required(VERSION 3.5)
project(proxemics_costmap_layer)

# Buscar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

# Agregar la biblioteca compartida
add_library(${PROJECT_NAME} SHARED src/proxemics_layer.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_costmap_2d
  pluginlib
  geometry_msgs
)

# Instalar la biblioteca
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

# Instalar el archivo XML del plugin
install(FILES proxemics_costmap_layer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
