cmake_minimum_required(VERSION 3.8)
project(diff_drive_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetObjectPose.srv"
  DEPENDENCIES geometry_msgs
)

# Build Behavior Tree Plugins
add_library(check_command_condition SHARED src/check_command_condition.cpp)
ament_target_dependencies(check_command_condition rclcpp behaviortree_cpp std_msgs nav2_behavior_tree)

add_library(get_object_pose_action SHARED src/get_object_pose_action.cpp)
ament_target_dependencies(get_object_pose_action rclcpp behaviortree_cpp geometry_msgs nav2_behavior_tree)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(get_object_pose_action "${cpp_typesupport_target}")

# (ament_python_install_package is automatically called by rosidl_generate_interfaces)

install(
  PROGRAMS 
    scripts/yolo_perception_node.py
    scripts/semantic_memory_node.py
    scripts/ui_node.py
    scripts/odom_tf_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS check_command_condition get_object_pose_action
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch urdf config worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
