cmake_minimum_required(VERSION 2.8.3)
project(test_ariac)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(catkin REQUIRED COMPONENTS
  ariac_example
  osrf_gear
)

# Used by test/
set(ROS_PACKAGE_NAME osrf_gear)
catkin_package(
  CATKIN_DEPENDS
  osrf_gear
  ariac_example
)

set(rostests
  ariac_rosapi.test
  ariac_example.test
  gripper.test
)

foreach(rostest ${rostests})
  # We don't set a timeout here because we trust rostest to enforce the
  # timeout specified in each .test file.
  add_test(${rostest} rostest ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
  # Check for test result file and create one if needed.  rostest can fail to
  # generate a file if it throws an exception.
  add_test(check_${rostest} rosrun rosunit check_test_ran.py --rostest ${ROS_PACKAGE_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${rostest})
endforeach()

# Install test files because we'll run tests from the install tree.  We could
# get away with running most of the test from the source tree, but it's clearer
# and safer to run them from the install tree.
install(FILES
  ${rostests}
  ariac_rosapi.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)

install(PROGRAMS
  ros_api_checker
  test_example_node.py
  test_gripper.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
