cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(ProximityRayPlugin SHARED ProximityRayPlugin.cc)
target_link_libraries(ProximityRayPlugin ${GAZEBO_LIBRARIES})

add_library(ConveyorBeltPlugin SHARED ConveyorBeltPlugin.cc)
target_link_libraries(ConveyorBeltPlugin ${GAZEBO_LIBRARIES})
