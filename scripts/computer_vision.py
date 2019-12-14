#!/usr/bin/env python
# Author: Senthil Palanisamy
'''
This node does all the computer vision part of the project. The list of
activities preformed by this node are
- detects AR tags for estimating camera pose
- detects red brick on the camera's field of view
- Converts pixel coordinates of pixel location to 3D world cooridnates location
  that can be fed for the trajectory planning pipeline.

SERVICES:
  + /blocks/next_pickup (blocks/GetBlockPosition) - A service for figuring out
                                                    the 3D coordinates of the
                                                    next block to pickup
'''

import time
import random

import rospy
import tf
import numpy as np
import rospkg
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Pose2D
from blocks.srv import GetBlockPosition
from cv_bridge import CvBridge, CvBridgeError


from pixel_frame_transform import *

class BlockLocaliser:
  '''
   A class for detecting and locating the 3D locations of the block
  '''
  def __init__(self):
    '''
    Initialise topics and some parameters used in the algorithm.
    '''
    camera_info = rospy.wait_for_message('/cameras/right_hand_camera/camera_info',
                                         CameraInfo)
    rospy.Service('/blocks/next_pickup', GetBlockPosition, 
                  self.get_block_position)
    self.K = np.array([401.157794, 0.000000, 646.813359, 0.000000, 399.765366, 
                       417.705109, 0.000000, 0.000000, 1.000000]).reshape(3, 3)
    self.listener = tf.TransformListener()

    # The list of all ar_markers visible in the camera's field of field
    # compare frame is the ar marker with respect to which a few algorithmic
    # workspace to pick up the block from are designed.
    self.all_marker_frame_name_list = ['ar_marker_0', 'ar_marker_1', 'ar_marker_2', 
                                       'ar_marker_3', 'ar_marker_4', 
                                       'ar_marker_5', 'ar_marker_6',
                                       'ar_marker_7', 'ar_marker_8']

    self.compare_frame= 'ar_marker_6'
    self.camera_frame_name = 'reference/right_hand_camera'


    # Images for displaying emojis during algorithmic processing.
    rospack = rospkg.RosPack()
    rospack.list()
    frowny_face_image = cv2.imread(rospack.get_path('blocks')+
                                                        '/images/frowny.jpeg')
    self.bridge = CvBridge()
    self.frowny_face = self.bridge.cv2_to_imgmsg(frowny_face_image, 
                                                 encoding="passthrough")
    self.camera_image_name = '/cameras/right_hand_camera/image'
    self.image_output = rospy.Publisher("/all_blocks/op_image", 
                                        Image, queue_size = 1)
    self.head_display = rospy.Publisher("/robot/xdisplay", 
                                        Image, queue_size = 1)
    self.ref_frame_name = 'world'


  def get_transform_between_frames(self, reference_frame, target_frame):
    '''
    This function returns the transformation between the two given frames
    reference_frame - A string specifying the desired reference frame as per
                      tf listener
    target_frame - A string specifying the desired target frame as per 
                   tf listener
    Returns:
    transformation - [translation_vector, rotation_in_quaternion_format]
    '''

    time_out = 0.3
    start_time = time.time()

    while(True):
      try:
        translation_vector, rotation_quaternions = self.listener.lookupTransform(
                                                  reference_frame,
                                                  target_frame,
                                                  rospy.Time(0))
        break
      except (tf.LookupException, tf.ConnectivityException, 
                                                   tf.ExtrapolationException):
        if((time.time()- start_time) > time_out):
          return None
    transformation = [translation_vector, rotation_quaternions]
    return transformation



  def get_all_block_poses(self, image):
    '''
    This function processes the given image and gives out the location 
    of the centers of all red blocks detected.
    Input:
    image - an opecv image for processig
    Returns:
    selected_points - A list of pixel points indicating the centers of 
                      all detected red blocks
    orientations - 0/1 indicating if the block's orientation is horizontal or
                   vertical. 0 means the block is horizontal and 1 means the
                   block is vertical
    '''
    image = image.copy()
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    lower_red = np.array([0,120,70], dtype = "uint8")
    upper_red = np.array([30,255,255], dtype = "uint8")

    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    output = cv2.bitwise_and(image, image, mask = mask1)


    lower_red = np.array([160,120,70], dtype = "uint8")
    upper_red = np.array([180,255,255], dtype = "uint8")

    mask2 = cv2.inRange(hsv, lower_red, upper_red)
    output = cv2.bitwise_and(image, image, mask = mask2)

    mask = mask1 + mask2

    im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)
    expected_area = 3000
    alpha = 0.8
    minimum_area = alpha * expected_area
    filtered_contours = filter(lambda x:cv2.contourArea(x) > minimum_area, 
                                                                     contours)

    if not filtered_contours:
      return [np.array([[-1], [-1]]), 0]

    selected_points = []
    x_points, y_points = [], []
    orientations = []
    self.all_blocks = []
    for selected_contour in filtered_contours:
      x,y,w,h = cv2.boundingRect(selected_contour)
      xc, yc = x + w/2, y + h/2
      cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
      self.all_blocks.append([x, y, x+w, y+h])
      x_points.append(xc)
      y_points.append(yc)
      if w > h:
        orientations.append(0)
      else:
        orientations.append(1)
    selected_points = np.array([x_points, y_points])

    op_image = self.bridge.cv2_to_imgmsg(image, encoding="passthrough")
    self.image_output.publish(op_image)
    return selected_points, orientations

  def send_target_block_image(self, image, index):
    '''
    Sends the display image with the selected block highlighted in a rectangle
    Inputs:
    image - an opencv image
    index - index indicting which block was selected among all the blocks
            that were detected
    '''
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
    '''
    Transforms the given point, which is represented in a given reference
    frame to a point in the destination frame.
    Input:
    transformation - Transformation, which represents the rotation of and
                     translation of the destination frame in the given reference
                     frame
                     [translation_vector, rotation in quaternion format]
    point - A point in the given reference frame.
    Return:
    point_in_target - The given point represented in the target frame. 
    '''

    translation_vector, rotation_quaternions = transformation
    transformation_matrix = tf.transformations.quaternion_matrix(
                                                         rotation_quaternions)
    transformation_matrix[:3, 3] = translation_vector
    point_in_target_frame = transformation_matrix.dot(point)
    return point_in_target_frame

  def get_block_position(self, data):
    '''
    TODO: This function has become too big at this stage. Replace this with
    modularised code later.
    A service call for returning the position of the next block to be picked up
    Highlevel overview and design decision of the function:-
    The function detects all blocks in the given image. All AR frames in the 
    given field of view of detected. The pixel co-ordinates are then converted
    to 3D world co-ordinates in terms of each of the AR frames. Now the points
    in  each of the AR frames are converted to baxter's world frame. Now the
    median of all these points are taken and given out as the target point
    where Baxter's hand should be positioned.
    Reason for using Multiple AR frames -A single AR frame is very unreliable
    and is badly affected by ligths and camera exposure. Multiple AR frame
    gives us a chance to work around this unrealiabitly issues.
    '''

    # Get image and detect the 
    image_in = rospy.wait_for_message(self.camera_image_name, Image)
    image = self.bridge.imgmsg_to_cv2(image_in, "bgr8")
    pixel_values, orientation = self.get_all_block_poses(image)
    block_pose = Pose2D()

    # If no blocks are detected, return large negative values for position
    if pixel_values[0][0] == -1 and pixel_values[1][0] == -1:
      block_pose.x = -1000
      block_pose.y = -1000
      block_pose.theta = -1000
      self.head_display.publish(self.frowny_face)
      return block_pose

    # Intialise dictionaries one for each marker
    points_in_frames = {}
    for marker_frame_name in self.all_marker_frame_name_list:
        points_in_frames[marker_frame_name] = []
    tag_camera_transfroms = {}
    tag_base_transforms = {}


    # Convert pixels to points in each frame
    for marker_frame_name in self.all_marker_frame_name_list:
      camera_to_tag_transform = self.get_transform_between_frames(
                                                       self.camera_frame_name, 
                                                       marker_frame_name)
      tag_camera_transfroms[marker_frame_name] = camera_to_tag_transform
      print('got transform')
      target_transform = self.get_transform_between_frames(
                                       self.ref_frame_name, marker_frame_name)
      if target_transform and camera_to_tag_transform:
        print(target_transform)
        tag_base_transforms[marker_frame_name] = target_transform
        point_in_ar_frame = transform_pixel_to_any_frame(pixel_values, 
                                              camera_to_tag_transform, self.K)
        points_in_frames[marker_frame_name] = point_in_ar_frame

    points_in_frames_unfiltered = points_in_frames.copy()
    compare_frame_points = points_in_frames[self.compare_frame]

    # Remove blocks which are already placed in goal destinations
    blocks_to_remove = []
    for index, point in enumerate(compare_frame_points):
      if point[1] < 0.02:
        blocks_to_remove.append(index)


    # If there are not valid blocks to pick up, return large negative numbers
    if len(compare_frame_points) - len(blocks_to_remove) < 0:
      block_pose.x = -1000
      block_pose.y = -1000
      block_pose.theta = -1000
      self.head_display.publish(self.frowny_face)

      return block_pose

    # Remove all blocks corresponding to the invalid blocks
    for frame_name, points in points_in_frames.items():
      new_points = [point_in_ar.tolist() for point_index, point_in_ar in 
                    enumerate(points) if point_index not in blocks_to_remove]
      points_in_frames[frame_name] = np.array(new_points)

    # Select a block at random and change pixels to coordinates in the ar frames
    selected_point_index = random.randint(0, 
                                  len(points_in_frames[self.compare_frame])-1)
    point_in_all_frames = []

    for frame_name, points in points_in_frames.items():
      if points.size:
        point_in_ar_frame = points[selected_point_index]
        target_transform = tag_base_transforms[frame_name]
        point_in_ar_frame_4d = np.array([point_in_ar_frame[0], 
                                         point_in_ar_frame[1],
                                         point_in_ar_frame[2], 1])

        point_in_op_frame = self.transform_point(point_in_ar_frame_4d, 
                                                 target_transform)
        point_in_all_frames.append(point_in_op_frame)

    # Take median of the point reprented in several AR frames and return that
    # as the position to pick up the block from
    point_in_all_frames = np.array(point_in_all_frames)
    final_point = np.median(point_in_all_frames, axis=0)
    selected_point_in_cmp_frame = points_in_frames[self.compare_frame]\
                                                  [selected_point_index]


    image_index = [np.array_equal(points_in_frames_unfiltered[self.compare_frame][i],
                                  selected_point_in_cmp_frame[:3])
                   for i in range(len(
                   points_in_frames_unfiltered[self.compare_frame]))].index(True)

    self.send_target_block_image(image, image_index)

    point_in_op_frame = self.transform_point(final_point, target_transform)
    block_pose.x = final_point[0]
    block_pose.y = final_point[1]
    block_pose.theta = orientation[image_index]
    return block_pose




def main():
  '''
  main function for running the node
  '''
  rospy.init_node('computer_vision')
  blocklocaliser = BlockLocaliser()

  blocklocaliser.get_block_position('')
  rospy.spin()

if __name__=='__main__':
   try:
       main()
   except rospy.ROSInterruptException:
       pass
