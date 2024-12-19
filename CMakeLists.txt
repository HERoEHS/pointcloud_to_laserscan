cmake_minimum_required(VERSION 3.5)
project(pointcloud_to_laserscan)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(laserscan_to_pointcloud SHARED
  src/laserscan_to_pointcloud_node.cpp)

rclcpp_components_register_node(laserscan_to_pointcloud
  PLUGIN "pointcloud_to_laserscan::LaserScanToPointCloudNode"
  EXECUTABLE laserscan_to_pointcloud_node)

ament_auto_add_library(pointcloud_to_laserscan SHARED
  src/pointcloud_to_laserscan_node.cpp)

rclcpp_components_register_node(pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE pointcloud_to_laserscan_node)


ament_auto_add_library(filtered_pointcloud_to_laserscan SHARED
  src/filtered_pointcloud_to_laserscan_node.cpp)

rclcpp_components_register_node(filtered_pointcloud_to_laserscan
  PLUGIN "pointcloud_to_laserscan::PointCloudToLaserScanNode"
  EXECUTABLE filtered_pointcloud_to_laserscan_node)

ament_auto_add_executable(dummy_pointcloud_publisher
  src/dummy_pointcloud_publisher.cpp
)

# 새로 추가한 sor_pointcloud_filter 노드를 위한 빌드 설정
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(sor_pointcloud_filter_node
  src/sor_pointcloud_filter.cpp
)
ament_target_dependencies(sor_pointcloud_filter_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
)

install(TARGETS sor_pointcloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
