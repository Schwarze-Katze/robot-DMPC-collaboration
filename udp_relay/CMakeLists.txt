cmake_minimum_required(VERSION 3.10)
project(udp_relay)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
nav_msgs
geometry_msgs
tf
dynamic_reconfigure
)
find_package(yaml-cpp REQUIRED)

generate_dynamic_reconfigure_options(
  config/init.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lib
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)


ADD_EXECUTABLE(master_relay 
  src/master.cpp
  src/udp_master.cpp
)

ADD_EXECUTABLE(slave_relay 
  src/slave.cpp
  src/udp_slave.cpp
)

catkin_install_python(PROGRAMS scripts/sim_remap.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


TARGET_LINK_LIBRARIES(master_relay
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)
TARGET_LINK_LIBRARIES(slave_relay
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)


