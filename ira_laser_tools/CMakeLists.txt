cmake_minimum_required(VERSION 3.5)
project(ira_laser_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(DEPS rclcpp sensor_msgs tf2 tf2_ros)

add_executable(laserscan_multi_merger src/laserscan_multi_merger.cpp)
ament_target_dependencies(laserscan_multi_merger ${DEPS})

add_executable(laserscan_virtualizer src/laserscan_virtualizer.cpp)
ament_target_dependencies(laserscan_virtualizer ${DEPS})

install(TARGETS laserscan_multi_merger laserscan_virtualizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
