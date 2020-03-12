cmake_minimum_required(VERSION 2.8.3)
project(mapping)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  interfaces
  roscpp
  rospy
  std_msgs
  visualization_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  DP.msg
  LC.msg
  PG.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  visualization_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    interfaces
    std_msgs
    visualization_msgs
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

add_executable(lcDetection src/lcDetection.cpp)
target_link_libraries(lcDetection ${catkin_LIBRARIES})
add_dependencies(lcDetection ${catkin_EXPORTED_TARGETS} mapping_generate_messages_cpp)

add_executable(pgOptimization src/pgOptimization.cpp)
target_link_libraries(pgOptimization ${catkin_LIBRARIES})
add_dependencies(pgOptimization ${catkin_EXPORTED_TARGETS} mapping_generate_messages_cpp)
