cmake_minimum_required(VERSION 3.10)
project(asv_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
