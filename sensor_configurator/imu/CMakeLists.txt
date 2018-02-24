cmake_minimum_required(VERSION 2.8.3)
project(imu)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  serial
  roscpp
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_test
  CATKIN_DEPENDS 
  serial
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
)


add_executable(imu_main
  src/IMUConfigurator.cpp
  src/imu_main.cpp
)
add_dependencies(imu_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_main
  ${catkin_LIBRARIES}
)

add_executable(covariance_changer
  src/covariance_changer.cpp
)
add_dependencies(covariance_changer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(covariance_changer
  ${catkin_LIBRARIES}
)