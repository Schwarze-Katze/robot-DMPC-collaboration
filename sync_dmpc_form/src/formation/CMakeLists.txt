cmake_minimum_required(VERSION 3.10)
project(formation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  mymsg
)

catkin_package(
CATKIN_DEPENDS roscpp message_runtime
)

LINK_DIRECTORIES( ${ipopt_LIBRARY_DIRS} )
include_directories(
  ${catkin_INCLUDE_DIRS}
)

SET(source_list
	src/main.cpp
)
#
ADD_EXECUTABLE(formation ${source_list})
#add_dependencies(formation mymsg_gencpp)
#
# libraries to be linked into the specified target,
# as determined by pkg-config for ipopt
TARGET_LINK_LIBRARIES(formation
    ${cppad_lib}
    /usr/local/lib/libipopt.so
    ${colpack_libs}
    ${catkin_LIBRARIES}
)


