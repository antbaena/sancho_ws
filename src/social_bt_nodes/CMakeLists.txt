cmake_minimum_required(VERSION 3.8)
project(social_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(people_msgs REQUIRED)
find_package(pluginlib REQUIRED)

add_library(social_bt_nodes SHARED
  src/is_human_blocking_condition.cpp
)

ament_target_dependencies(social_bt_nodes
  rclcpp
  nav2_behavior_tree
  geometry_msgs
  people_msgs
  pluginlib
)

pluginlib_export_plugin_description_file(BT::ConditionNode social_bt_nodes_plugins.xml)

install(TARGETS social_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES social_bt_nodes_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
