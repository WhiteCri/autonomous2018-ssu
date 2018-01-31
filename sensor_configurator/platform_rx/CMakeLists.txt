cmake_minimum_required(VERSION 2.8.3)
project(platform_rx)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  serial
)

add_message_files(FILES PlatformRX_msg.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    serial
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(platform_rx_main_node
  src/platform_rx_main_node.cpp 
)

target_link_libraries(platform_rx_main_node
  ${catkin_LIBRARIES}
)

#add_executable(platformRx_test_main_node
#  src/platformRx_test_main_node.cpp
#)
#add_dependencies(platformRx_test_main_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})