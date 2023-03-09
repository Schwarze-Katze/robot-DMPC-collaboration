cmake_minimum_required(VERSION 3.10)
project(mymsg)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs  
  message_generation
)


add_message_files(
  FILES
  neighborpos.msg
  refpos.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(

CATKIN_DEPENDS roscpp message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)




