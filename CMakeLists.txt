cmake_minimum_required(VERSION 3.8)
project(ur_picking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(moveit_simple_controller_manager REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(robotiq_controllers REQUIRED)
find_package(robotiq_description REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPose.action"
  DEPENDENCIES geometry_msgs moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

add_executable(ur_picking_node
  src/ur_picking_node.cpp
)

ament_target_dependencies(ur_picking_node
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_kinematics
  moveit_planners_ompl
  moveit_ros_visualization
  moveit_simple_controller_manager
  rclcpp
  rclcpp_action
  rclcpp_components
  std_msgs
  moveit_msgs
)

target_link_libraries(
  ur_picking_node
  "${cpp_typesupport_target}"
)

add_executable(goal_receive_node
  src/goal_receive_node.cpp
)

ament_target_dependencies(goal_receive_node
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  std_msgs
  moveit_msgs
  action_msgs
)

target_link_libraries(
  goal_receive_node
  "${cpp_typesupport_target}"
)

install(TARGETS
  ur_picking_node
  goal_receive_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config urdf srdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
