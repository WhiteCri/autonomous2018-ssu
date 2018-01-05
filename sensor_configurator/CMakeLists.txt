cmake_minimum_required(VERSION 2.8.3)
project(sensor_configurator)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)

catkin_package(
  CATKIN_DEPENDS
    serial
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(imu_test_main_node
  src/imu_test_main_node.cpp
  src/IMUConfigurator.cpp
  src/SensorConfigurator.cpp
  src/SerialCommunicator.cpp
)

target_link_libraries(imu_test_main_node
  ${catkin_LIBRARIES}
)
