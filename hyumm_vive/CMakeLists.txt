cmake_minimum_required(VERSION 3.0.2)
project(hyumm_vive)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp tf geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(vive_world_anchor_node src/vive_world_anchor_node.cpp)
target_link_libraries(vive_world_anchor_node ${catkin_LIBRARIES})

add_executable(mobile_base_node src/mobile_base_node.cpp)
target_link_libraries(mobile_base_node ${catkin_LIBRARIES})

install(TARGETS vive_world_anchor_node mobile_base_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
