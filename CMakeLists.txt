cmake_minimum_required(VERSION 2.8.3)
project(discretized_movement)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wunused")

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_move_group
  moveit_ros_perception
  moveit_ros_visualization
)

find_package(Boost REQUIRED COMPONENTS
  system
)

add_message_files(FILES
  interact.msg
  movement.msg
  robotstate.msg
  worldstate.msg
)

add_action_files(FILES
  Interact.action
  Move.action
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  actionlib_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_move_group
  moveit_ros_perception
  moveit_ros_visualization
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_directories(${Boost_LIBRARY_DIR})

add_executable(movement_server
  src/movement_server.cpp
)

target_link_libraries(movement_server ${catkin_LIBRARIES})

add_dependencies(
  movement_server
  ${discretized_movement_EXPORTED_TARGETS}
)
