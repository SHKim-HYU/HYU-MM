cmake_minimum_required(VERSION 3.5)
project(hyumm_nrt)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(hyumm_vive REQUIRED)
find_package(Threads REQUIRED)

set(HYUMM_NRT_DEPS
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  hyumm_vive
)

include_directories(include)

# Primary node: in-process ViveLocalizer + self-healing XDDP bridge.
add_executable(hyumm_vive_xddp_node src/hyumm_vive_xddp_node.cpp)
ament_target_dependencies(hyumm_vive_xddp_node ${HYUMM_NRT_DEPS})

# Legacy single-file vive -> XDDP bridge.
add_executable(hyumm_scan_xddp_node src/hyumm_scan_xddp_node.cpp)
ament_target_dependencies(hyumm_scan_xddp_node ${HYUMM_NRT_DEPS})

# RT odom -> /odom (background reader thread).
add_executable(hyumm_xddp_node src/hyumm_xddp_node.cpp)
ament_target_dependencies(hyumm_xddp_node ${HYUMM_NRT_DEPS})
target_link_libraries(hyumm_xddp_node Threads::Threads)

# RT odom -> /odom + /cmd_vel -> XDDP.
add_executable(multi_hyumm_xddp_node src/multi_hyumm_xddp_node.cpp)
ament_target_dependencies(multi_hyumm_xddp_node ${HYUMM_NRT_DEPS})
target_link_libraries(multi_hyumm_xddp_node Threads::Threads)

install(TARGETS
  hyumm_vive_xddp_node
  hyumm_scan_xddp_node
  hyumm_xddp_node
  multi_hyumm_xddp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Export the header-only XDDP transport (xddp_ros.h / xddp_packet.h) so other
# packages (e.g. hyumm_bridge) can include the self-healing XddpLink + packet schema.
install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

ament_package()
