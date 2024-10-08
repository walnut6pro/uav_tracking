cmake_minimum_required(VERSION 2.8.3)
project(tianracer_navigation)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base
  tf
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS visualization_msgs
#  INCLUDE_DIRS include
#  LIBRARIES tianracer
#  CATKIN_DEPENDS geometry_msgs move_base roscp rospy std_msgs visualization_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/tianracer.cpp
# )

add_executable(plan src/plan.cpp)
target_link_libraries(plan ${catkin_LIBRARIES})

add_executable(build_plan src/build_plan.cpp)
target_link_libraries(build_plan ${catkin_LIBRARIES})
# Pure Pursuit Node
# add_executable(Pure_Pursuit src/Pure_Pursuit.cpp
#         include/tianracer_navigation/Pure_Pursuit.h)
# target_link_libraries(Pure_Pursuit ${catkin_LIBRARIES})
add_executable(Pure_Pursuit src/Second_Pure_Pursuit.cpp
        include/tianracer_navigation/Pure_Pursuit.h)
target_link_libraries(Pure_Pursuit ${catkin_LIBRARIES})