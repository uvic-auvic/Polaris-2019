cmake_minimum_required(VERSION 2.8.3)
project(nav)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  peripherals
  dsp
  geometry_msgs
)

find_package(
  peripherals
)

## Generate messages in the 'msg' folder
 add_message_files(
  FILES
    request.msg
 )

## Generate services in the 'srv' folder
add_service_files(
  FILES
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    peripherals
 )

catkin_package(
  #LIBRARIES rov
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)


include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${dsp_INCLUDE_DIRS}
)

