cmake_minimum_required(VERSION 2.8.3)
project(cute_servo_controllers)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  actionlib
  dynamixel_motor
  dynamixel_controllers
  dynamixel_driver
  std_msgs
  trajectory_msgs
  diagnostic_msgs
  dynamixel_msgs
  control_msgs
)

install(PROGRAMS
  nodes/controller_manager.py
  nodes/controller_spawner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
