cmake_minimum_required(VERSION 3.8)
project(trajectory_tracker)

# 设置 C++ 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 包含目录
include_directories(
  include
)

# 创建可执行文件
add_executable(trajectory_tracker_node
  src/trajectory_tracker_node.cpp
)

# 链接库
ament_target_dependencies(trajectory_tracker_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

# 安装可执行文件
install(TARGETS
  trajectory_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)


# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出包
ament_package()
