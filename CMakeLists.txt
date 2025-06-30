cmake_minimum_required(VERSION 3.5)
project(pointcloud_concatenate_ros2)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pointcloud_concatenate_ros2 SHARED
  src/pointcloud_concat_node.cpp)

rclcpp_components_register_node(pointcloud_concatenate_ros2
  PLUGIN "pointcloud_concatenate::PointCloudConcatNode"
  EXECUTABLE pointcloud_concat_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
    launch
)
