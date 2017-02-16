set(rostests
  ariac_rosapi.test
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
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
