cmake_minimum_required(VERSION 2.8.3)
project(lane_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)
add_message_files(
   FILES
  dataSize.msg
  coordinateMsg.msg
  vecMsg.msg

 )
generate_messages(DEPENDENCIES
  std_msgs
)
set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -O2 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")
catkin_package(
INCLUDE_DIRS include
LIBRARIES preprocessing
CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs std_msgs roscpp
DEPENDS system_lib
)

find_package(OpenCV)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(avipublisher
  src/avipublisher.cpp
)
target_link_libraries(avipublisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(lane_detection
  src/lane_detection.cpp
  src/preLaneDetect.cpp
)

target_link_libraries(lane_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(lane_detection2
  src/lane_detection2.cpp
)

target_link_libraries(lane_detection2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)