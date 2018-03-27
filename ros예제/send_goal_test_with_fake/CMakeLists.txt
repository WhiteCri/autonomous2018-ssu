cmake_minimum_required(VERSION 2.8.3)
project(send_goal_test_with_fake)


find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  move_base_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  LIBRARIES ros_tutorials_action
  CATKIN_DEPENDS actionlib actionlib_msgs roscpp
)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_executable(move_base_server
  src/move_base_server.cpp
)
add_dependencies(move_base_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base_server
  ${catkin_LIBRARIES}
)

add_executable(move_base_client
  src/move_base_client.cpp
)
add_dependencies(move_base_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base_client
  ${catkin_LIBRARIES}
)