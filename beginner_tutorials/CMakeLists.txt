cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Num.msg
 )

 add_service_files(
   FILES  
   AddTwoInts.srv
 )


generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

#catkin_install_python(PROGRAMS scripts/server.py scripts/add_two_ints_client.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
