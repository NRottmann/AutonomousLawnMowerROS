cmake_minimum_required(VERSION 2.8.3)
project(control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  interfaces
  localization
  roscpp
  rospy
  serial
  std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    interfaces
    localization
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
)

add_executable(teleop src/teleop.cpp)
add_executable(randomWalk src/randomWalk.cpp)
add_executable(wallFollower src/wallFollower.cpp)
add_executable(pdControl src/pdControl.cpp)

target_link_libraries(teleop ${catkin_LIBRARIES})
target_link_libraries(randomWalk ${catkin_LIBRARIES})
target_link_libraries(wallFollower ${catkin_LIBRARIES})
target_link_libraries(pdControl ${catkin_LIBRARIES})

add_dependencies(teleop ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp localization_generate_messages_cpp)
add_dependencies(randomWalk ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp localization_generate_messages_cpp)
add_dependencies(wallFollower ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp localization_generate_messages_cpp)
add_dependencies(pdControl ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp localization_generate_messages_cpp)
