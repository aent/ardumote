cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(ardumote
  SRCS main.cpp PPMGenerator.cpp ${ROS_LIB_DIR}/time.cpp
  BOARD uno
  PORT /dev/ttyUSB0
)

