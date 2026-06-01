cmake_minimum_required(VERSION 3.5)
project(hyumm_vive)

# Header-only localization library (ViveLocalizer). No nodes -- the hyumm_nrt
# node includes this header and does the TF broadcast + XDDP bridge.
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(tf2 tf2_geometry_msgs geometry_msgs)

ament_package()
