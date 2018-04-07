cmake_minimum_required(VERSION 2.8.3)
project(highlevel_controller)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
)

add_message_files(FILES
  params.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)



include_directories(
  include ${catkin_INCLUDE_DIRS}
)




add_executable(highlevel_controller
  src/highlevel_controller.cpp
  src/base_parameter.cpp
)
add_dependencies(highlevel_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(highlevel_controller
  ${catkin_LIBRARIES}
)

