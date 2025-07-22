cmake_minimum_required(VERSION 3.0.2)
project(arm_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  rosserial_arduino
  rosserial_client
  std_msgs
)

add_message_files(
  FILES
  arm_master_comm.msg
  arm_dynamixel_state.msg
  arm_joint_state.msg
  arm_robot_state.msg
  arm_master_comm_int.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arm_msgs
  CATKIN_DEPENDS message_generation roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)