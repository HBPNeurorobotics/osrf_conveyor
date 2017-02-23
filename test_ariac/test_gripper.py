#!/usr/bin/env python

from __future__ import print_function

import sys
import time
import unittest

from ariac_example import ariac_example
import rospy
import rostest


class Tester(unittest.TestCase):

    def test_gripper(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)
        time.sleep(1.0)

        # Pre-defined pose that puts the gripper in contact with a part.
        self._send_arm_to_part()

        # Enable the gripper so that it picks up the part.
        success = ariac_example.control_gripper(True)
        self.assertTrue(success, 'Gripper not successfully controlled')
        time.sleep(1.0)
        self.assertTrue(
            self.comp_class.current_gripper_state.enabled, 'Gripper not successfully enabled')
        self.assertTrue(
            self.comp_class.current_gripper_state.attached, 'Part not successfully attached')

        # Move the part over the tray using a pre-defined sequence of poses.
        self._send_arm_to_tray()
        self.assertTrue(
            self.comp_class.current_gripper_state.enabled, 'Gripper no longer enabled')
        self.assertTrue(
            self.comp_class.current_gripper_state.attached, 'Part no longer attached')

        # Disable the gripper so that it drops the part.
        success = ariac_example.control_gripper(False)
        self.assertTrue(success, 'Gripper not successfully controlled')
        time.sleep(1.0)
        self.assertFalse(
            self.comp_class.current_gripper_state.enabled, 'Gripper not successfully disabled')
        self.assertFalse(
            self.comp_class.current_gripper_state.attached, 'Part not successfully dettached')

        time.sleep(1.0)

    def _send_arm_to_part(self):
        positions = [1.85, 0.35, -0.38, 2.76, 3.67, -1.51, 0.0]
        self.comp_class.send_arm_to_state(positions)
        time.sleep(1.5)

    def _send_arm_to_tray(self):
        trajectory = [
            [1.76, 0.28, -1.38, 2.76, 3.27, -1.51, 0.0],
            [1.76, 0.38, -1.38, 1.5, 3.27, -1.51, 0.0],
            [1.76, 2.06, -1.38, 1.5, 3.27, -1.51, 0.0],
            [1.76, 2.06, -0.63, 1.5, 3.27, -1.51, 0.0],
        ]
        for positions in trajectory:
            self.comp_class.send_arm_to_state(positions)
            time.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node('test_gripper', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(12.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_gripper', Tester, sys.argv)
