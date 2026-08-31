cmake_minimum_required(VERSION 3.8)
project(assembly_with_links)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  meshes
  urdf
  launch
  config 
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
