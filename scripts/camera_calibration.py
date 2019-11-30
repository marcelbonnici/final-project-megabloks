# Full credits of this code lie with OpenCV tutorials

import numpy as np
import cv2
import glob
import yaml

import json
import os

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

rows = 9
columns = 6

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((rows * columns,3), np.float32)
objp[:,:2] = np.mgrid[0:rows,0:columns].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('./calibration_images/selected_images/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (rows,columns),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        print('here')
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (rows, columns), corners2,ret)
        cv2.imshow('img',img)
        cv2.waitKey(500)

cv2.destroyAllWindows()
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
print(mtx)

img = cv2.imread(images[0])
h,  w = img.shape[:2]
newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
camera_parameters = {}

config_file_path = './config/camera_parameters.yaml' 
is_file_exisis = os.path.isfile(config_file_path)

if(is_file_exisis):
  with open(config_file_path) as in_file:
    camera_parameters = yaml.full_load(in_file)
else:
  camera_parameters = {}


#camera_parameters += [{'broken_baxter':{'left_camera':{'distortion':dist.tolist(), 
#                                                       'intrinsic':newcameramtx.tolist()}}}]
camera_parameters['broken_baxter'] = {'left_camera':{'distortion': dist.tolist()}}
camera_parameters['broken_baxter']['left_camera']['intrinsic']= newcameramtx.tolist()

with open(config_file_path, 'w') as out_file:
    documents = yaml.dump(camera_parameters, out_file)



# undistort
# View results
# dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
# 
# # crop the image
# x,y,w,h = roi
# dst = dst[y:y+h, x:x+w]
# cv2.imwrite('calibresult.png',dst)
