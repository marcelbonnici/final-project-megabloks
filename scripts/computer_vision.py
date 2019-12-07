#!/usr/bin/env python

import time
import random

import rospy
import tf
import numpy as np
import rospkg

from pixel_frame_transform import *
from sensor_msgs.msg import Image


from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Pose2D
from blocks.srv import GetBlockPosition
from cv_bridge import CvBridge, CvBridgeError

class BlockLocaliser:
  def __init__(self):
    camera_info = rospy.wait_for_message('/cameras/right_hand_camera/camera_info', CameraInfo)
    rospy.Service('/blocks/next_pickup', GetBlockPosition, self.get_block_position)
    self.K = np.array(camera_info.K).reshape(3,3)
    self.listener = tf.TransformListener()
    self.marker_frame_name = 'ar_marker_6'
    self.camera_frame_name = 'reference/right_hand_camera'
    self.ref_frame_name = 'reference/base'
    #frowny_face_image = cv2.imread('./images/frowny.jpeg')

    rospack = rospkg.RosPack()
    rospack.list() 
    frowny_face_image = cv2.imread(rospack.get_path('blocks')+'/images/frowny.jpeg')

    self.bridge = CvBridge()
    self.frowny_face = self.bridge.cv2_to_imgmsg(frowny_face_image, encoding="passthrough")
    # self.camera_image_name = '/cameras/right_hand_camera/camera'
    self.camera_image_name = '/cameras/right_hand_camera/image'
    self.image_output = rospy.Publisher("/all_blocks/op_image", Image, queue_size = 1)
    self.head_display = rospy.Publisher("/robot/xdisplay", Image, queue_size = 1)


  def get_transform_between_frames(self, reference_frame, target_frame):

    time_out = 10
    start_time = time.time()

    while(True):
      try:
        translation_vector, rotation_quaternions = self.listener.lookupTransform(
                                                  reference_frame, 
                                                  target_frame, 
                                                  rospy.Time(0))
        break
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        if((time.time()- start_time) > time_out):
          raise Exception('time-out: transformations not published as required')
    transformation = [translation_vector, rotation_quaternions]
    if(transformation):
      print('got transformation')
    return transformation


  def get_all_block_poses(self, image):
    image = image.copy()
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
  
    lower_red = np.array([0,120,70], dtype = "uint8")
    upper_red = np.array([10,255,255], dtype = "uint8")
  
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    output = cv2.bitwise_and(image, image, mask = mask1)
  
  
    lower_red = np.array([170,120,70], dtype = "uint8")
    upper_red = np.array([180,255,255], dtype = "uint8")
  
    mask2 = cv2.inRange(hsv, lower_red, upper_red)
    output = cv2.bitwise_and(image, image, mask = mask2)
  
    mask = mask1 + mask2
  
    im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE) 
    expected_area = 4000
    alpha = 0.8
    minimum_area = alpha * expected_area
    filtered_contours = filter(lambda x:cv2.contourArea(x) > minimum_area, contours)
  
    if not filtered_contours:
      return [np.array([[-1], [-1]]), 0]
  
    selected_points = []
    x_points, y_points = [], []
    orientations = []
    self.all_blocks = []
    for selected_contour in filtered_contours:
      # Reason: we need the bottom most point of the object(y+h) but the
      # center of the object in terms of the object width
      x,y,w,h = cv2.boundingRect(selected_contour)
      # This might be a simplification. Check how well this works
      #points_only = np.vstack(selected_contour).squeeze()
      #sorted_points = sorted(points_only, key = lambda x:x[1])
      #xc, yc = sorted_points[-1]
      xc, yc = x + w/2, y + h/2
      cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
      self.all_blocks.append([x, y, x+w, y+h])
      x_points.append(xc)
      y_points.append(yc)
      if w > h:
        orientations.append(0)
      else:
        orientations.append(1)
    #y_points = [y_points[index] for index in np.argsort(x_points)]
    #x_points = list(sorted(x_points))
    selected_points = np.array([x_points, y_points])

    op_image = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
    self.image_output.publish(op_image)


    #cv2.imshow('image', image)
    #cv2.waitKey(0)

    return selected_points, orientations 

  def send_target_block_image(self, image, index):
    image = image.copy()  
    x1, y1, x2, y2 = self.all_blocks[index] 
    cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,0),10)
    # display resolution of baxter screen
    display_width, display_height =1024, 600
    newimg = cv2.resize(image,(display_width, display_height))
    newimg = cv2.flip( newimg, -1 )
    newimg = self.bridge.cv2_to_imgmsg(newimg, encoding="passthrough")
    self.head_display.publish(newimg)



  def transform_point(self, point, transformation):

    translation_vector, rotation_quaternions = transformation
    transformation_matrix = tf.transformations.quaternion_matrix(rotation_quaternions) 
    transformation_matrix[:3, 3] = translation_vector
    point_in_target_frame = transformation_matrix.dot(point)
    return point_in_target_frame
  
  def get_block_position(self, data):

    image_in = rospy.wait_for_message(self.camera_image_name, Image)
    image = self.bridge.imgmsg_to_cv2(image_in, "bgr8")
    pixel_values, orientation = self.get_all_block_poses(image)
    block_pose = Pose2D()

    if pixel_values[0][0] == -1 and pixel_values[1][0] == -1:
      block_pose.x = -1000
      block_pose.y = -1000
      block_pose.theta = -1000
      self.head_display.publish(self.frowny_face)
      return block_pose

    # check if the order is correct and what you expect
    # camera_to_tag_transform = self.get_transform_between_frames(self.marker_frame_name, self.camera_frame_name)
    camera_to_tag_transform = self.get_transform_between_frames(self.camera_frame_name, self.marker_frame_name)
    print('got transform')
    target_transform = self.get_transform_between_frames(self.ref_frame_name, self.marker_frame_name)
    # pixel_values = [pixel_location[0], pixel_location[1]]
    point_in_ar_frame = transform_pixel_to_any_frame(pixel_values, camera_to_tag_transform, self.K)  
    filtered_points = []
    for point in point_in_ar_frame:
        if point[0] < 0.235:
            filtered_points.append([point[0], point[1], point[2]])

    if not filtered_points:
      block_pose.x = -1000
      block_pose.y = -1000
      block_pose.theta = -1000
      #print(type(self.frowny_face)
      self.head_display.publish(self.frowny_face)

      return block_pose

    print(point_in_ar_frame)
    index = random.randint(0, len(filtered_points)-1)
    selected_point = np.array([filtered_points[index][0], filtered_points[index][1],
                               filtered_points[index][2], 1])
    self.send_target_block_image(image, index)

    point_in_op_frame = self.transform_point(selected_point, target_transform) 
    block_pose.x = point_in_op_frame[0]
    block_pose.y = point_in_op_frame[1]
    block_pose.theta = orientation[index]
    return block_pose

    


def main():
  # pixel_location = get_block_from_images(image)
  rospy.init_node('computer_vision')
  blocklocaliser = BlockLocaliser()
  
  blocklocaliser.get_block_position('')
  rospy.spin()

if __name__=='__main__':
  main()
