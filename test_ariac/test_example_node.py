#!/usr/bin/env python

from __future__ import print_function

import sys
import time
import unittest

from ariac_example import ariac_example
from std_msgs.msg import Float32
import rospy
import rostest


class Tester(unittest.TestCase):

    def comp_score_callback(self, msg):
        self.current_comp_score = msg.data

    def test_example_node(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)

        self.current_comp_score = None
        self.comp_state_sub = rospy.Subscriber(
            "/ariac/current_score", Float32, self.comp_score_callback)

        # Starting the competition will cause parts from the order to be spawned on AGV1
        self._test_start_comp()
        time.sleep(1.0)
        self._test_order_reception()

        # Pre-defined initial pose because sometimes the arm starts "droopy"
        self._send_arm_to_initial_pose()
        self._test_send_arm_to_zero_state()

        self._test_agv_control()
        time.sleep(0.5)
        self._test_comp_end()

    def _test_start_comp(self):
        success = ariac_example.start_competition()
        self.assertTrue(success, 'Failed to start the competition')
        time.sleep(0.5)
        self.assertTrue(
            self.comp_class.current_comp_state == 'go', 'Competition not in "go" state')

    def _test_order_reception(self):
        self.assertEqual(len(self.comp_class.received_orders), 1)

    def _send_arm_to_initial_pose(self):
        positions = [1.51, 0.0, -1.12, 3.14, 3.77, -1.51, 0.0]
        self.comp_class.send_arm_to_state(positions)
        time.sleep(1.0)

    def _test_send_arm_to_zero_state(self):
        self.comp_class.send_arm_to_state([0] * len(self.comp_class.arm_joint_names))
        time.sleep(1.5)
        error = 0
        for position in self.comp_class.current_joint_state.position:
            error += abs(position - 0.0)
        self.assertTrue(error < 0.5, 'Arm was not properly sent to zero state')

    def _test_agv_control(self):
        success = ariac_example.control_agv(1, 'order_0_kit_0')
        self.assertTrue(success, 'Failed to control AGV')

    def _test_comp_end(self):
        self.assertTrue(
            self.comp_class.current_comp_state == 'done', 'Competition not in "done" state')
        self.assertTrue(
            self.current_comp_score == 6.0,
            'Something went wrong in the scoring. Current score: ' + str(self.current_comp_score))


if __name__ == '__main__':
    rospy.init_node('test_example_node', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(10.0)
    print('OK, starting test.')

    rostest.run('osrf_gear', 'test_example_node', Tester, sys.argv)
