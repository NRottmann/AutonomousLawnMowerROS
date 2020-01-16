cmake_minimum_required(VERSION 2.8.3)
project(localization)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_C_COMPILER "gcc")

add_compile_options(-std=c++11)

## link_directories(/usr/local/include)

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
  Pose.msg
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


add_executable(kalmanFilter src/kalmanFilter.cpp)
target_link_libraries(kalmanFilter ${catkin_LIBRARIES})
add_dependencies(kalmanFilter ${catkin_EXPORTED_TARGETS} localization_generate_messages_cpp)

add_executable(particleFilter src/particleFilter.cpp)
target_link_libraries(particleFilter ${catkin_LIBRARIES})
add_dependencies(particleFilter ${catkin_EXPORTED_TARGETS} localization_generate_messages_cpp)
