cmake_minimum_required(VERSION 3.0.2)
project(hyumm_vive)

add_compile_options(-std=c++14)

# Header-only localization library (ViveLocalizer). No nodes -- the hyumm_nrt
# node links this and does the TF broadcast + XDDP bridge.
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf geometry_msgs
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
