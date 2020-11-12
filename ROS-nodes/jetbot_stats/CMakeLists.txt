cmake_minimum_required(VERSION 3.0.2)
project(jetbot_stats)
# version 0.0.1

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  jetbot_msg
  std_msgs
  std_srvs
)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES creating_a_ros_library
    CATKIN_DEPENDS roscpp std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(jetbot_stats_class src/jetbot_stats_class.cpp)

## Declare a C++ executable
add_executable(jetbot_stats src/jetbot_stats.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(
	jetbot_stats
	jetbot_stats_class
	ina219
	i2c
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############

