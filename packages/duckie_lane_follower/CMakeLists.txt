cmake_minimum_required(VERSION 3.0.2)
project(duckie_lane_follower)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  duckietown_msgs
  cv_bridge
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  src/lane_follower_node.py
  src/command_gateway.py
  src/camera_gateway.py
  src/camera_capture.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
