cmake_minimum_required(VERSION 2.8.3)
project(osrf_gear)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  roscpp
  std_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY
  launch worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Configure the setup script
catkin_add_env_hooks(99_osrf_gear_setup
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
  SHELLS sh)
