cmake_minimum_required(VERSION 3.8)
project(relocalization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED) 
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED) 

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

# 包含PCL头文件
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 编译节点
add_executable(relocalization_node src/relocalization_node.cpp)
ament_target_dependencies(relocalization_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  pcl_conversions
)
target_link_libraries(relocalization_node ${PCL_LIBRARIES})  # 链接PCL库

install(TARGETS
  relocalization_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
