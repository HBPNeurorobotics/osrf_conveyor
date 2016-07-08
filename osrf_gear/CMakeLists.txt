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

install(PROGRAMS
  scripts/run_gazebo
  scripts/run_gazebo_paused
  scripts/run_gzclient
  scripts/run_gzserver
  scripts/run_gzserver_callgrind
  scripts/run_gzserver_cheats
  scripts/run_gzserver_gdb
  scripts/run_gzserver_memcheck
  scripts/run_gzserver_perf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

# Configure the setup script
configure_file(${PROJECT_SOURCE_DIR}/cmake/setup.sh.in
               ${PROJECT_BINARY_DIR}/setup.sh @ONLY)
catkin_add_env_hooks(setup SHELLS sh DIRECTORY ${PROJECT_BINARY_DIR})
