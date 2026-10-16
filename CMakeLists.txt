cmake_minimum_required(VERSION 3.0.2)
project(ntrip_gnss_relay)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(
  PROGRAMS ntrip_gnss_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
