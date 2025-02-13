cmake_minimum_required(VERSION 3.0.2)
project(msd700_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
	contoller_manager
	hardware_interface
)

catkin_package(
	CATKIN_DEPENDS roscpp controller_manager hardware_interface
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)