cmake_minimum_required(VERSION 3.8)
project(my_robot_teleop)

# Add compile options, required for all compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# Create a library for the component
add_library(${PROJECT_NAME}_component SHARED
  src/joystick_servo_example.cpp)

# Since C++17 is used, you might need to request it explicitly (if your code uses C++17 features)
target_compile_features(${PROJECT_NAME}_component PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  sensor_msgs
  geometry_msgs
  control_msgs
  std_srvs
  moveit_msgs
  rclcpp_components)

# This line registers the component in the ament index and sets the plugin to be loadable on runtime
rclcpp_components_register_nodes(${PROJECT_NAME}_component "my_robot_teleop::JoyToServoPub")

# Install the library for the component
install(TARGETS ${PROJECT_NAME}_component
  DESTINATION lib)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export information to downstream packages
ament_package()
