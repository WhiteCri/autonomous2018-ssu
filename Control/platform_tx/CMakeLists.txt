cmake_minimum_required(VERSION 2.8.3)
project(platform_tx)

find_package(catkin REQUIRED
  ackermann_msgs
)

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

add_executable(platform_tx
  src/platform_tx.cpp
)

add_dependencies(platform_tx 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(platform_tx
  ${catkin_LIBRARIES}
)