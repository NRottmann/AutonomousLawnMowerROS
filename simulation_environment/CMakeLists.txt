cmake_minimum_required(VERSION 2.8.3)
project(simulation_environment)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  interfaces
  message_generation
)

find_package(Eigen3 REQUIRED)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    interfaces
    message_runtime
  DEPENDS
    EIGEN3
)

# Depending on system install of Gazebo
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${src}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Gazebo Plugins
add_library(odometry_plugin SHARED src/odometry_plugin.cpp)
add_library(chlorophyllSensor_plugin SHARED src/chlorophyllSensor_plugin.cpp)

target_link_libraries(odometry_plugin ${GAZEBO_LIBRARIES})
target_link_libraries(chlorophyllSensor_plugin ${GAZEBO_LIBRARIES})

add_dependencies(odometry_plugin ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp)
add_dependencies(chlorophyllSensor_plugin ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp)

# ROS node
add_executable(sensor_transform src/sensor_transform.cpp)
add_executable(odometry_transform src/odometry_transform.cpp)

target_link_libraries(sensor_transform ${catkin_LIBRARIES})
target_link_libraries(odometry_transform ${catkin_LIBRARIES})

add_dependencies(sensor_transform ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp)
add_dependencies(odometry_transform ${catkin_EXPORTED_TARGETS} interfaces_generate_messages_cpp)



# Gazebo set flags
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

