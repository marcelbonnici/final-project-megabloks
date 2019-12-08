#!/usr/bin/env python
import unittest
import sys
from sensor_msgs.msg import CameraInfo, Image
PKG = 'blocks'
class CamTestCase(unittest.TestCase):
    def test_camera(self):
        self.camera_info = rospy.wait_for_message('/cameras/right_hand_camera/camera_info', CameraInfo)
        self.assertEqual(self.camera_info.width, 800)

if __name__=='__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_cam', CamTestCase)
    #rospy.init_node('test_cam')
    #rostest.rosrun('blocks', 'test_cam', CamTestCase)#unittest.main()
