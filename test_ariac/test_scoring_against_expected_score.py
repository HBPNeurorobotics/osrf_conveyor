#!/usr/bin/env python

from __future__ import print_function

import sys
import time

from test_example_node import ExampleNodeTester
import rospy
import rostest


class ScoringTester(ExampleNodeTester):

    def test(self):
        expectedScore = float(sys.argv[1])
        rospy.loginfo('Using expected score of: ' + str(expectedScore))
        self.prepare_tester()

        # Starting the competition will cause parts from the order to be spawned on AGV1
        self._test_start_comp()

        # Submit the tray
        self._test_agv_control()
        time.sleep(5.0)

        self.assertTrue(
            self.current_comp_score == expectedScore,
            'Something went wrong in the scoring. Expected score of ' + str(expectedScore) +
                ' but received: ' + str(self.current_comp_score))


if __name__ == '__main__':
    rospy.init_node('test_scoring_after_waiting', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(10.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_scoring_after_waiting', ScoringTester, sys.argv)
