cmake_minimum_required(VERSION 2.8.3)
project(fcl_playing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  interactive_markers
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_package()


###########
## Build ##
###########

set(FCL_Library_Path /usr/local/lib/)
set(FCL_Include_Path /usr/local/include/fcl/)

set(Sejong_Library_Path /usr/local/lib/SDC_Toolkit/)
set(Sejong_Include_Path /usr/local/include/SDC_Toolkit/)
set(Sejong_Eigen_Path ~/Sejong_Dynamic_Control_Toolkit/ExternalSource)

find_library(my_libfcl NAMES libfcl.so PATHS ${FCL_Library_Path} REQUIRED)
find_library(my_libccd NAMES libccd.so PATHS ${FCL_Library_Path} REQUIRED)

find_library(my_libValkyrie_model NAMES libValkyrie_model.so PATHS ${Sejong_Library_Path} REQUIRED)
find_library(SJUtils NAMES libSJutils.so PATHS ${Sejong_Library_Path}  REQUIRED)
find_library(SJrbdl NAMES libSJrbdl.so PATHS ${Sejong_Library_Path}  REQUIRED)


################
## Executable ##
################

include_directories(include ${catkin_INCLUDE_DIRS})
#include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${FCL_Include_Path})
include_directories(${Sejong_Include_Path})
include_directories(${Sejong_Eigen_Path})

set(${FCL_TESTING}_SOURCES 
    src/fcl_tests.cpp
    src/test_fcl_utility.h
    src/markerGeneration.cpp
    src/RobotCollisionChecker.hpp
    src/helpfulMath.cpp
  )

add_executable(fcl_tests ${${FCL_TESTING}_SOURCES})
add_executable(collisionMarker src/interactiveMarker.cpp)


target_link_libraries(fcl_tests 
    ${catkin_LIBRARIES} 
    ${my_libfcl} 
    ${my_libccd} 
    ${my_libValkyrie_model}
    ${SJUtils}
    ${SJrbdl}
    )

target_link_libraries(collisionMarker ${catkin_LIBRARIES})
#    ${my_libValkyrie_model}
