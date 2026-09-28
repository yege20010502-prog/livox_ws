cmake_minimum_required(VERSION 3.5)
project(dog_control_bridge)

# 强制使用C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(dog_control_bridge 
    src/dog_control_bridge.cpp
)

target_include_directories(dog_control_bridge PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
)

target_link_libraries(dog_control_bridge
    ${rclcpp_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
    pthread
)

install(TARGETS dog_control_bridge
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
