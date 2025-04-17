cmake_minimum_required(VERSION 3.8)
project(xarm_vision_grasp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(xarm_msgs REQUIRED)
find_package(xarm_api REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED)
find_package(image_geometry REQUIRED) 
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Grasp.srv"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable
add_executable(xarm_vision_grasp_node src/red_block_detector.cpp)

add_executable(vision_node src/vision_node.cpp)
ament_target_dependencies(vision_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
)
add_executable(tf_test_node src/tf_test_node.cpp)
ament_target_dependencies(tf_test_node rclcpp tf2_ros geometry_msgs)
install(TARGETS tf_test_node DESTINATION lib/${PROJECT_NAME})

# add_executable(arm_pickup_node src/arm_pickup_node.cpp)
# ament_target_dependencies(arm_pickup_node rclcpp tf2_ros geometry_msgs moveit_ros_planning_interface tf2_geometry_msgs)

add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node rclcpp std_msgs geometry_msgs)

add_executable(arm_control_node src/arm_control_node.cpp)
ament_target_dependencies(arm_control_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
)

rosidl_target_interfaces(control_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(arm_control_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(xarm_vision_grasp_node ${OpenCV_LIBRARIES})
ament_target_dependencies(xarm_vision_grasp_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  moveit_ros_planning_interface
  xarm_msgs
  xarm_api
  image_geometry
)




install(TARGETS
  control_node
  arm_control_node
  xarm_vision_grasp_node
  tf_test_node
  # arm_pickup_node
  vision_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()