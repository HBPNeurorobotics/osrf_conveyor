#!/usr/bin/env python

from __future__ import print_function

import sys
import time
import unittest

from ariac_example import ariac_example
import rospy
import rostest


class Tester(unittest.TestCase):

    def test_example_node(self):
        self.comp_class = ariac_example.MyCompetitionClass()
        ariac_example.connect_callbacks(self.comp_class)
        self._test_start_comp()
        time.sleep(1.0)
        self._test_order_reception()

    def _test_start_comp(self):
        success = ariac_example.start_competition()
        self.assertTrue(success, 'Failed to start the competition')

    def _test_order_reception(self):
        self.assertEqual(len(self.comp_class.received_orders), 1)


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
