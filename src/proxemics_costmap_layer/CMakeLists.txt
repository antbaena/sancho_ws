cmake_minimum_required(VERSION 3.5)
project(proxemics_costmap_layer)

set(lib_name ${PROJECT_NAME}_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Buscar dependencias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib,
    geometry_msgs)


# Agregar la biblioteca compartida
add_library(${lib_name} SHARED src/proxemics_layer.cpp)
include_directories(include)


# Instalar la biblioteca
install(TARGETS ${lib_name}
        DESTINATION lib)

# Instalar el archivo XML del plugin
pluginlib_export_plugin_description_file(nav2_costmap_2d proxemics_costmap_layer_plugin.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
