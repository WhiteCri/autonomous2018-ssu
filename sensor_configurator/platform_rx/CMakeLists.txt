cmake_minimum_required(VERSION 2.8.3)
project(platform_rx)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  platform_rx_msg
)

catkin_package(
  CATKIN_DEPENDS
    serial
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(platform_rx_main_node
  src/platform_rx_main_node.cpp 
)

add_dependencies(platform_rx_main_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(platform_rx_main_node
  ${catkin_LIBRARIES}
)