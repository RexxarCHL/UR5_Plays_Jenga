cmake_minimum_required(VERSION 2.8.3)

# include_directories(${ROS_LIB_DIR})
include_directories(~/Arduino/libraries/ros_lib)
link_directories(${ARDUINO_SDK_PATH}/libraries)
link_directories(~/Arduino/libraries)

generate_arduino_firmware(end_effector_control
  SRCS end_effector_control_v2.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyACM0
)
