cmake_minimum_required(VERSION 2.8.3)
project(simulation_912)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)



## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(simulation src/simulation.cpp)
target_link_libraries(simulation ${catkin_LIBRARIES})


