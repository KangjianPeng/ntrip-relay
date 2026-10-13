cmake_minimum_required(VERSION 3.0.2)
project(ntrip_gnss_relay)

find_package(catkin REQUIRED COMPONENTS
  gnss_comm
  rospy
)

catkin_package(
  CATKIN_DEPENDS gnss_comm rospy
)

catkin_install_python(
  PROGRAMS ntrip_gnss_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
