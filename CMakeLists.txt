cmake_minimum_required(VERSION 3.8)
project(ntrip_gnss_relay)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS ntrip_gnss_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ntrip_gnss_relay
)

ament_package()
