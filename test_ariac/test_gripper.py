#!/usr/bin/env python

from __future__ import print_function

import sys
import time
import unittest

from ariac_example import ariac_example
import rospy
import rostest

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class Tester(unittest.TestCase):

    def test_gripper(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)
        time.sleep(1.0)
        self._send_arm_to_part()
        success = ariac_example.control_gripper(True)
        self.assertTrue(success, 'Gripper not successfully controlled')
        time.sleep(1.0)
        self.assertTrue(
            self.comp_class.current_gripper_state.enabled, 'Gripper not successfully enabled')
        self.assertTrue(
            self.comp_class.current_gripper_state.attached, 'Part not successfully attached')

    def _send_arm_to_part(self):
        positions = [1.85, 0.35, -0.38, 2.76, 3.67, -1.51, 0.0]
        self.comp_class.send_arm_to_state(positions)
        time.sleep(1.5)


if __name__ == '__main__':
    rospy.init_node('test_gripper', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(10.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_gripper', Tester, sys.argv)
