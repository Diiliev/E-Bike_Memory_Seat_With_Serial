cmake_minimum_required(VERSION 3.0.2)
project(ebms_with_serial)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY actions
  FILES adjustSeatHeight.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp rospy std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(ebmsActionServer src/server/EbmsActionServer.cpp)
add_executable(ebmsActionClient src/client/EbmsActionClient.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ebmsActionServer
  ${catkin_LIBRARIES}
)
target_link_libraries(
  ebmsActionClient
  ${catkin_LIBRARIES}
)

add_dependencies(
  ebmsActionServer
  ebmsActionClient
  ${ebms_with_serial_EXPORTED_TARGETS}
)
