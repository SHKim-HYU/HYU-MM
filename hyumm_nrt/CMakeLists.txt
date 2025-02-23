cmake_minimum_required(VERSION 3.0.2)
project(hyumm_nrt)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hyumm_xddp_node src/hyumm_xddp_node.cpp)
add_executable(multi_hyumm_xddp_node src/multi_hyumm_xddp_node.cpp)
target_link_libraries(hyumm_xddp_node ${catkin_LIBRARIES})
target_link_libraries(multi_hyumm_xddp_node ${catkin_LIBRARIES})
