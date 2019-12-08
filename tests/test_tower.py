#! /usr/bin/env python
import unittest
import rospy

class TowerCase(unittest.TestCase):
    def test_param_loaded(self):
        tower = rospy.get_param('/tower', None)
        self.assertIsNotNone(tower)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('blocks', 'test_tower', TowerCase)
