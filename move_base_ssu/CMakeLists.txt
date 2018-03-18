cmake_minimum_required(VERSION 2.8.3)
project(move_base_ssu)


find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
    CATKIN_DEPENDS actionlib actionlib_msgs 
)


include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(send_goal
  src/send_goal.cpp
)
add_dependencies(send_goal ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(send_goal
  ${catkin_LIBRARIES}
)