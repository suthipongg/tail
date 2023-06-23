cmake_minimum_required(VERSION 3.0.2)
project(convert_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs  
  sensor_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  joint_state_to_pos_array src/joint_state_to_pos_array.cpp
)

target_link_libraries(
  joint_state_to_pos_array
  ${catkin_LIBRARIES}
)
  
# add_dependencies(
#   # joint_state_to_pos_array
# )