cmake_minimum_required(VERSION 3.8)
project(ntrip_um960_relay)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS ntrip_um960_relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ntrip_um960_relay
)

ament_package()
