cmake_minimum_required(VERSION 3.0.2)
project(msd700_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs msd700_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
