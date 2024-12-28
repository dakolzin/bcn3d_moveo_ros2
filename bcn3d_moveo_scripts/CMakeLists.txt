cmake_minimum_required(VERSION 3.5)
project(bcn3d_moveo_scripts)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Generate ROS 2 messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmJointState.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

# Build executables
add_executable(moveit_convert src/moveit_convert.cpp)
ament_target_dependencies(moveit_convert rclcpp sensor_msgs rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(moveit_convert ${cpp_typesupport_target})

# Install executables
install(TARGETS moveit_convert
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(go_go_joint1 src/go_go_joint1.cpp)
ament_target_dependencies(go_go_joint1 rclcpp moveit_ros_planning_interface tf2_ros tf2_geometry_msgs moveit_msgs)

# Install executables
install(TARGETS go_go_joint1
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(go_go_generate src/go_go_generate.cpp)
ament_target_dependencies(go_go_generate rclcpp moveit_ros_planning_interface tf2_ros tf2_geometry_msgs moveit_msgs)

# Install executables
install(TARGETS go_go_generate
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(go_go_generate_seven src/go_go_generate_seven.cpp)
ament_target_dependencies(go_go_generate_seven rclcpp moveit_ros_planning_interface tf2_ros tf2_geometry_msgs moveit_msgs)

# Install executables
install(TARGETS go_go_generate_seven
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(go_go_to_the_point src/go_go_to_the_point.cpp)
ament_target_dependencies(go_go_to_the_point rclcpp moveit_ros_planning_interface tf2_ros tf2_geometry_msgs moveit_msgs geometry_msgs)

# Install executables
install(TARGETS go_go_to_the_point
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(to_the_point src/to_the_point.cpp)
ament_target_dependencies(to_the_point rclcpp moveit_ros_planning_interface tf2_ros tf2_geometry_msgs moveit_msgs geometry_msgs)

# Install executables
install(TARGETS to_the_point
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(go_go_joint2 src/go_go_joint2.cpp)
ament_target_dependencies(go_go_joint2 rclcpp moveit_ros_planning_interface tf2_ros tf2_geometry_msgs moveit_msgs)

# Install executables
install(TARGETS go_go_joint2
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(go_go_joint3 src/go_go_joint3.cpp)
ament_target_dependencies(go_go_joint3 rclcpp moveit_ros_planning_interface tf2_ros tf2_geometry_msgs moveit_msgs)

# Install executables
install(TARGETS go_go_joint3
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(go_go_joint4 src/go_go_joint4.cpp)
ament_target_dependencies(go_go_joint4 rclcpp moveit_ros_planning_interface tf2_ros tf2_geometry_msgs moveit_msgs)

# Install executables
install(TARGETS go_go_joint4
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(go_go_test_ml src/go_go_test_ml.cpp)
ament_target_dependencies(go_go_test_ml rclcpp moveit_ros_planning_interface tf2_ros tf2_geometry_msgs moveit_msgs)

# Install executables
install(TARGETS go_go_test_ml
  DESTINATION lib/${PROJECT_NAME}
)

# Install ROS 2 messages
install(DIRECTORY
  msg
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.msg"
)

# Export ROS 2 package information
ament_export_dependencies(
  rclcpp
  moveit_ros_planning_interface
  std_msgs
  sensor_msgs
)

ament_package()
