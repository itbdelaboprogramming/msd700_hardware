cmake_minimum_required(VERSION 3.0.2)
project(msd700_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  msd700_msgs
  controller_manager
	hardware_interface
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs msd700_msgs controller_manager hardware_interface
)

include_directories(
  

  ${catkin_INCLUDE_DIRS}
)

add_executable(msd700_hw_interface src/msd700_hw_interface.cpp)
target_link_libraries(msd700_hw_interface ${catkin_LIBRARIES})
add_dependencies(msd700_hw_interface ${catkin_EXPORTED_TARGETS})
