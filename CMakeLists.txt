cmake_minimum_required(VERSION 3.16)
project(container_unloading_sim_benchmark)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

add_library(contact_gripper_plugin SHARED
  plugins/src/contact_gripper_plugin.cpp)

target_include_directories(contact_gripper_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/plugins/include)

target_link_libraries(contact_gripper_plugin
  gz-sim::gz-sim
  gz-plugin::register
  rclcpp::rclcpp
  ${std_msgs_TARGETS})

install(PROGRAMS
  scripts/extract_metrics.py 
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS contact_gripper_plugin
  LIBRARY DESTINATION lib)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/plugins/contact_gripper_plugin.dsv.in")

install(DIRECTORY launch world urdf config
  DESTINATION share/${PROJECT_NAME})

ament_package()