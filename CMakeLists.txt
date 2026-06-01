cmake_minimum_required(VERSION 3.5)
project(hyumm_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(hyumm_nrt REQUIRED)
find_package(hyumm_ocs2 REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# hyumm_ocs2 exports Boost::log/log_setup/system/filesystem as interface link deps
find_package(Boost REQUIRED COMPONENTS system filesystem log log_setup)

set(HYUMM_BRIDGE_DEPS
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  hyumm_nrt
  hyumm_ocs2
  ocs2_core
  ocs2_mpc
  ocs2_oc
  ocs2_ros_interfaces
  ocs2_msgs
)

add_executable(hyumm_bridge_node src/HyummBridgeNode.cpp)
ament_target_dependencies(hyumm_bridge_node ${HYUMM_BRIDGE_DEPS})
target_link_libraries(hyumm_bridge_node Eigen3::Eigen)

install(TARGETS hyumm_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
