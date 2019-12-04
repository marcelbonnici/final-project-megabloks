# Author: Senthil Palanisamy
import math
import time

import numpy as np
import cv2
import rospy
import roslib
import tf



def imgPointToWorldCoord(XY, R,
                         tvec, cameraMatrix, zconst=0):
    '''
    @returns 3d object coords

    :param (ix,iy): list of 2d points (x,y) in image
    :param zconst: height above image plane (if 0, than on image plane)

    http://answers.opencv.org/question/62779/image-coordinate-to-world-coordinate-opencv/
    and
    http://stackoverflow.com/questions/12299870/computing-x-y-coordinate-3d-from-image-point
    '''
    (ix, iy) = XY
    uvPoint = np.array([ix.ravel(), iy.ravel(), np.ones(
        shape=ix.size)]).reshape(3, ix.size)

    iR = R.T
    iC = np.linalg.inv(cameraMatrix)

    t = iR.dot(iC).dot(uvPoint)
    t2 = iR.dot(tvec)
    s = (zconst + t2[2]) / t[2]

    objP = (iR.dot(s * iC.dot(uvPoint) - tvec))
    return objP



def euler_angles_to_rotationMatrix(theta) :
    '''
    Inputs:-
    theta =[thetax, thetay, thetaz] (angle order is roll pitch yaw)
    Returns:-
    R - Rotation matrix corresponding to the euler sequence
    '''


    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])

    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])

    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])

    R = np.dot(R_z, np.dot( R_y, R_x ))
    return R




def transform_pixel_to_footprint_frame(point, transformation):
  '''
  Transforms a pixel co-oridantes to 3D distance in terms of the ground plane
  assuming that the foot print frame lies on the ground
  Inputs:-
  point - list containg all pixel points to be transformed
  point = [[point1x, point2x, point3x......], [point1y, point2y,point3y.....]]
  transformation - rotation and translation of the footprint frame with 
                   respect to the camera frame
  transformation = [[tx,ty,tz],[q1,q2,q3,q4]] where t* corresponds to translation
                   co-ordinates and q* corresponds to quaternion values
  Returns:-
  pt_in_foot_print_frame - 3D location of points corresponding to pixel locations
  pt_in_foot_print_frame = [[p1x,p2x,p3x....], [p1y,p2y,p3y...],[p1z,p2z,p3z..]]
  where p*z will be zero or very close to zero since the 3D point lies on 
  the ground plane
  '''

  translation_vector, rotation_quaternions = transformation

  axis_angles = tf.transformations.euler_from_quaternion(rotation_quaternions)
  rotation_matrix = euler_angles_to_rotationMatrix(axis_angles)
  translation_vector = np.array(translation_vector) * 1000
  translation_vector = translation_vector.reshape(3,1)
  # TODO: Move this intrinsic matrix to a config file after deciding a location
  # for config files.
  intrinsic_matrix = np.array([[1518.3315230088997, 0.0, 633.3583896757208],
                               [0.0, 1517.6029896637713, 519.0442053995603], 
                               [0.0, 0.0, 1.0]])
  pt_in_foot_print = imgPointToWorldCoord(point, rotation_matrix, translation_vector,
                                          intrinsic_matrix, 0)
  x, y , z = pt_in_foot_print
  y = [y[index] for index in np.argsort(x)]
  z = [z[index] for index in np.argsort(x)]
  x = list(sorted(x))
  points_3d = np.array(map(list, zip(x,y,z)))
  # Reason: measurement in calibration process is fully done in mm but the 
  # result is needed in meters.
  pt_in_foot_print = points_3d / 1000.0
  return pt_in_foot_print


if __name__=='__main__':
    pixel_to_camera_coordinate_conversion([1,1])
