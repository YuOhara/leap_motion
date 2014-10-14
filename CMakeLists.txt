cmake_minimum_required(VERSION 2.8.3)
project(leap_motion)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation sensor_msgs)

add_message_files(
    FILES
    leap.msg
    leapros.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES leap_motion
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  /home/ohara/LeapDeveloperKit_2.1.4+22333_linux/LeapSDK/include
)

install(PROGRAMS
  scripts/leap_interface.py scripts/sender.py scripts/subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(leap_sender src/leap_sender.cpp)
 
target_link_libraries(leap_sender
  ${catkin_LIBRARIES} /home/ohara/LeapDeveloperKit_2.1.4+22333_linux/LeapSDK/lib/x64/libLeap.so
  )
