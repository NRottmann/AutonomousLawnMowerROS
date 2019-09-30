cmake_minimum_required(VERSION 2.8.3)
project(interfaces)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_C_COMPILER "gcc")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Odometry.msg
  Sensor.msg
  IMU.msg
  Control.msg
  Decawave.msg
  Bumper.msg
  Chlorophyll.msg
  Capacity.msg
  Radar.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    geometry_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
)

add_executable(motorInterface src/motorInterface.cpp)
add_executable(sensorInterface src/sensorInterface.cpp)
add_executable(i2cInterface src/i2cInterface.cpp)
add_executable(decawaveInterface src/decawaveInterface.cpp)
add_executable(bumperInterface src/bumperInterface.cpp)
add_executable(radarInterface src/radarInterface.cpp)
add_executable(capacityInterface src/capacityInterface.cpp)

target_link_libraries(motorInterface ${catkin_LIBRARIES})
target_link_libraries(sensorInterface ${catkin_LIBRARIES})
target_link_libraries(i2cInterface ${catkin_LIBRARIES})
target_link_libraries(decawaveInterface ${catkin_LIBRARIES})
target_link_libraries(bumperInterface ${catkin_LIBRARIES})
target_link_libraries(radarInterface ${catkin_LIBRARIES})
target_link_libraries(capacityInterface ${catkin_LIBRARIES})

add_dependencies(motorInterface ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp)
add_dependencies(sensorInterface ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp)
add_dependencies(i2cInterface ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp)
add_dependencies(decawaveInterface ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp)
add_dependencies(bumperInterface ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp)
add_dependencies(radarInterface ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp)
add_dependencies(capacityInterface ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp)
