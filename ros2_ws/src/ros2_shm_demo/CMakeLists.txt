cmake_minimum_required(VERSION 3.8)
project(ros2_shm_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(shm_publisher src/publisher.cpp)
ament_target_dependencies(shm_publisher rclcpp std_msgs)

add_executable(shm_subscriber src/subscriber.cpp)
ament_target_dependencies(shm_subscriber rclcpp std_msgs)

install(TARGETS
  shm_publisher
  shm_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()


