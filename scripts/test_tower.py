#! /usr/bin/env python

import unittest
import rospy
import rostest


class MyTestCase(unittest.TestCase):
    def test_param_loaded(self):
        tower = rospy.get_param('/tower', None)
        self.assertIsNotNone(tower)


if __name__ == '__main__':
    rostest.rosrun('tutorial', 'test_params', MyTestCase)
