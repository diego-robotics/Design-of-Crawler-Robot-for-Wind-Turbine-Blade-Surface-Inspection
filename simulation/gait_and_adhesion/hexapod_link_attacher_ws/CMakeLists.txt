cmake_minimum_required(VERSION 3.8)
project(hexapod_link_attacher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(linkattacher_msgs REQUIRED)

add_library(hexapod_link_attacher SHARED
  src/hexapod_link_attacher.cpp
)

target_include_directories(hexapod_link_attacher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(hexapod_link_attacher
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "linkattacher_msgs"
)

install(TARGETS hexapod_link_attacher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(hexapod_link_attacher)
ament_export_dependencies(gazebo_dev gazebo_ros rclcpp linkattacher_msgs)

ament_package()
