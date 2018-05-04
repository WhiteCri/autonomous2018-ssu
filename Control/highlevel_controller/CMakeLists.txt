cmake_minimum_required(VERSION 2.8.3)
project(highlevel_controller)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  actionlib
  actionlib_msgs
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
  actionlib 
  actionlib_msgs 
)



include_directories(
  include ${catkin_INCLUDE_DIRS}
)




add_executable(highlevel_controller
  src/highlevel_controller.cpp
  src/base_parameter.cpp
  src/StateHandler.cpp
  src/condition.cpp
  src/HybridAutomata.cpp
  src/goalSender.cpp
)
add_dependencies(highlevel_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(highlevel_controller
  ${catkin_LIBRARIES}
)

add_executable(fake_param_setter
  src/fake_param_setter.cpp
)
add_dependencies(fake_param_setter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(fake_param_setter
  ${catkin_LIBRARIES}
)