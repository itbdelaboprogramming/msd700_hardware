cmake_minimum_required(VERSION 3.0.2)
project(msd700_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY urdf meshes launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)