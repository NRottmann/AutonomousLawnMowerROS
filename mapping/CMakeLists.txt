cmake_minimum_required(VERSION 2.8.3)
project(mapping)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  interfaces
  roscpp
  rospy
  std_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  DP.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    interfaces
    std_msgs
    message_runtime
  DEPENDS
    EIGEN3
)

## Include Directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(dpGeneration src/dpGeneration.cpp)
target_link_libraries(dpGeneration ${catkin_LIBRARIES})
add_dependencies(dpGeneration ${catkin_EXPORTED_TARGETS} mapping_generate_messages_cpp)
