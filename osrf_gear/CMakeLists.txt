cmake_minimum_required(VERSION 2.8.3)
project(osrf_gear)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS
  gazebo
  gazebo_plugins
  roscpp
  std_msgs
  message_generation
)

########################
## Message generation ##
########################

add_message_files(DIRECTORY msg
  FILES
  VacuumGripperState.msg
)

add_service_files(DIRECTORY srv
  FILES
  VacuumGripperControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Create the libVacuumGripperPlugin.so library.
set(vacuum_gripper_plugin_name VacuumGripperPlugin)
add_library(${vacuum_gripper_plugin_name} src/VacuumGripperPlugin.cc)
target_link_libraries(${vacuum_gripper_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
install(TARGETS ${vacuum_gripper_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Create the libROSVacuumGripperPlugin.so library.
set(ros_vacuum_gripper_plugin_name ROSVacuumGripperPlugin)
add_library(${ros_vacuum_gripper_plugin_name} src/ROSVacuumGripperPlugin.cc)
target_link_libraries(${ros_vacuum_gripper_plugin_name}
  ${catkin_LIBRARIES}
  ${vacuum_gripper_plugin_name}
  ${GAZEBO_LIBRARIES}
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
)
add_dependencies(${ros_vacuum_gripper_plugin_name}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
install(TARGETS ${ros_vacuum_gripper_plugin_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

#############
## Install ##
#############

# Install all the world files
install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds)

# Install all the model files
install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models)

# Install all the launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Configure the setup script
catkin_add_env_hooks(99_osrf_gear_setup
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/cmake
  SHELLS sh)
