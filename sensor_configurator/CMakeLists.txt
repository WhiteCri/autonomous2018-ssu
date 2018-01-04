cmake_minimum_required(VERSION 2.8.3)
project(sensor_configurator)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    SerialCommunicator
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(test_main_node src/test_main_node.cpp src/SerialCommunicator.cpp)

target_link_libraries(test_main_node
  ${catkin_LIBRARIES}
)
