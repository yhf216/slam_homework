cmake_minimum_required(VERSION 3.8)
project(map_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_srvs REQUIRED)

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



add_executable(pcd_saver_node src/pcd_saver_node.cpp)
ament_target_dependencies(pcd_saver_node
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  std_srvs
)

add_executable(map_publisher_node src/map_publisher_node.cpp)
ament_target_dependencies(map_publisher_node
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2_ros
  geometry_msgs
)
target_link_libraries(map_publisher_node
  pcl_common
  pcl_filters
  pcl_io
)

install(TARGETS
  pcd_saver_node
  map_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()