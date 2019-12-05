import time

import rospy
import tf
import numpy as np

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
    self.marker_frame_name = 'ar_marker_8'
    self.camera_frame_name = 'reference/right_hand_camera'
    self.output_frame_name = 'reference/base'
    # self.camera_image_name = '/cameras/right_hand_camera/camera'
    self.camera_image_name = '/cameras/right_hand_camera/image'
    self.bridge = CvBridge()


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
  
    if not contours:
      return [np.array([-1, -1]), 0]
  
    selected_points = []
    x_points, y_points = [], []
    orientations = []
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
      x_points.append(xc)
      y_points.append(yc)
      if w > h:
        orientations.append(0)
      else:
        orientations.append(90)
    y_points = [y_points[index] for index in np.argsort(x_points)]
    x_points = list(sorted(x_points))
    selected_points = np.array([x_points, y_points])

    cv2.imshow('image', image)
    cv2.waitKey(0)

    return selected_points, orientations 




  def transform_point(point, transform):

    translation_vector, rotation_quaternions = transformation
    transformation_matrix = tf.quaternion_matrix(rotation_quaternions) 
    transformation_matrix[:, :3] = translation_vector
    point_in_target_frame = translation_matrix.dot(point)
  
  def get_block_position(self):

    image_in = rospy.wait_for_message(self.camera_image_name, Image)
    image = self.bridge.imgmsg_to_cv2(image_in, "bgr8")
    pixel_values, orientation = self.get_all_block_poses(image)
    # check if the order is correct and what you expect
    camera_to_tag_transform = self.get_transform_between_frames(self.marker_frame_name, self.camera_frame_name)
    target_transform = self.get_transform_between_frames(self.output_frame_name, self.camera_frame_name)
    # pixel_values = [pixel_location[0], pixel_location[1]]
    point_in_ar_frame = transform_pixel_to_any_frame(pixel_values, camera_to_tag_transform, self.K)  
    index = random.randint(0, len(point_in_ar_frame)-1)
    selected_point = [point_in_ar_frame[index], point_in_ar_frame[index]]

    point_in_op_frame = transform_point(selected_point, target_transform) 
    block_pose = Pose2D()
    block_pose.x = point_in_op_frame[0]
    block_pose.y = point_in_op_frame[1]
    block_pose.theta = orientation[index]
    return block_pose

    


def main():
  image = cv2.imread('/home/senthilpalanisamy/work/courses/embedded_system_me_495/practice_ws/rethink_ws/src/final-project-megabloks/images/selected_image/left0000.jpg')
  # pixel_location = get_block_from_images(image)
  rospy.init_node('computer_vision')
  blocklocaliser = BlockLocaliser()
  blocklocaliser.get_block_position()
  rospy.spin()

if __name__=='__main__':
  main()
