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
    #self.K = np.array(camera_info.K).reshape(3,3)
    # self.K = np.array([406.688830, 0.000000, 653.829535, 0.000000, 405.951986, 423.000525, 0.000000, 0.000000, 1.000000]).reshape(3, 3)
    self.K = np.array([412.472805, 0.000000, 656.708584, 0.000000, 412.891416, 418.279946, 0.000000, 0.000000, 1.000000]).reshape(3, 3)
    #self.K = np.array([401.157794, 0.000000, 646.813359, 0.000000, 399.765366, 
    #                   417.705109, 0.000000, 0.000000, 1.000000]).reshape(3, 3)
    self.listener = tf.TransformListener()

    self.all_marker_frame_name_list = ['ar_markere_0', 'ar_marker_1', 'ar_marker_2', 'ar_marker_3',
                                      'ar_marker_4', 'ar_marker_5', 'ar_marker_6',
                                   'ar_marker_7', 'ar_marker_8']

    self.compare_frame= 'ar_marker_6'
    #self.marker_frame_name = 'ar_marker_2'
    self.camera_frame_name = 'reference/right_hand_camera'
    #self.ref_frame_name = 'reference/base'
    self.ref_frame_name = 'world'
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

    time_out = 0.3
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
          return None
    transformation = [translation_vector, rotation_quaternions]
    if(transformation):
      print('got transformation')
    return transformation


  def get_all_block_poses(self, image):
    image = image.copy()
    #clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    #image = clahe.apply(image)
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
    points_in_frames = {}
    for marker_frame_name in self.all_marker_frame_name_list:
        points_in_frames[marker_frame_name] = []
    tag_camera_transfroms = {}
    tag_base_transforms = {}


    for marker_frame_name in self.all_marker_frame_name_list:
      camera_to_tag_transform = self.get_transform_between_frames(self.camera_frame_name, marker_frame_name)
      tag_camera_transfroms[marker_frame_name] = camera_to_tag_transform
      print('got transform')
      target_transform = self.get_transform_between_frames(self.ref_frame_name, marker_frame_name)
      if target_transform and camera_to_tag_transform:
        print(target_transform)
        tag_base_transforms[marker_frame_name] = target_transform
        # pixel_values = [pixel_location[0], pixel_location[1]]
        point_in_ar_frame = transform_pixel_to_any_frame(pixel_values, camera_to_tag_transform, self.K)
        points_in_frames[marker_frame_name] = point_in_ar_frame

    points_in_frames_unfiltered = points_in_frames.copy()
    compare_frame_points = points_in_frames[self.compare_frame]

    blocks_to_remove = []
    for index, point in enumerate(compare_frame_points):
      if point[1] < 0.02:
        blocks_to_remove.append(index)


    if len(compare_frame_points) - len(blocks_to_remove) < 0:
      block_pose.x = -1000
      block_pose.y = -1000
      block_pose.theta = -1000
      #print(type(self.frowny_face)
      self.head_display.publish(self.frowny_face)

      return block_pose

    for frame_name, points in points_in_frames.items():
      new_points = [point_in_ar.tolist() for point_index, point_in_ar in enumerate(points)
                    if point_index not in blocks_to_remove]
      points_in_frames[frame_name] = np.array(new_points)




    # print(point_in_ar_frame)
    selected_point_index = random.randint(0, len(points_in_frames[self.compare_frame])-1)
    point_in_all_frames = []

    for frame_name, points in points_in_frames.items():
      if points.size:
        point_in_ar_frame = points[selected_point_index]
        target_transform = tag_base_transforms[frame_name]
        point_in_ar_frame_4d = np.array([point_in_ar_frame[0], point_in_ar_frame[1],
                                         point_in_ar_frame[2], 1])

        point_in_op_frame = self.transform_point(point_in_ar_frame_4d, target_transform)
        point_in_all_frames.append(point_in_op_frame)



    point_in_all_frames = np.array(point_in_all_frames)
    final_point = np.median(point_in_all_frames, axis=0)
    selected_point_in_cmp_frame = points_in_frames[self.compare_frame][selected_point_index]


    image_index = [np.array_equal(points_in_frames_unfiltered[self.compare_frame][i],
                                  selected_point_in_cmp_frame[:3])
                                  for i in range(len(points_in_frames_unfiltered[self.compare_frame]))].index(True)

    self.send_target_block_image(image, image_index)

    point_in_op_frame = self.transform_point(final_point, target_transform)
    block_pose.x = final_point[0]
    block_pose.y = final_point[1]
    block_pose.theta = orientation[image_index]
    return block_pose




def main():
  # pixel_location = get_block_from_images(image)
  rospy.init_node('computer_vision')
  blocklocaliser = BlockLocaliser()

  blocklocaliser.get_block_position('')
  rospy.spin()

if __name__=='__main__':
   try:
       main()
   except rospy.ROSInterruptException:
       pass
