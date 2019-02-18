# https://github.com/nullboundary/CharucoCalibration

import numpy as np
import cv2
import argparse
import sys
import yaml
import numpy as np
import time

def saveCameraParams(filename,imageSize,cameraMatrix,distCoeffs,totalAvgErr):

   print(cameraMatrix)

   calibration = {'camera_matrix': cameraMatrix.tolist(),'distortion_coefficients': distCoeffs.tolist()}

   calibrationData = dict(
       image_width = imageSize[0],
       image_height = imageSize[1],
       camera_matrix = dict(
         rows = cameraMatrix.shape[0],
         cols = cameraMatrix.shape[1],
         dt = 'd',
         data = cameraMatrix.tolist(),
         ),
       distortion_coefficients = dict(
           rows = disCoeffs.shape[0],
           cols = disCoeffs.shape[1],
           dt = 'd',
           data = disCoeffs.tolist(),
           ),
       avg_reprojection_error = totalAvgErr,
   )

   with open(filename,'w') as outfile:
       yaml.dump(calibrationData,outfile)


parser = argparse.ArgumentParser()

parser.add_argument("-f", "--file", help="ouput calibration filename",default="calibration.yml")
parser.add_argument("-s", "--size", help="size of squares in meters",type=float, default="0.035")
args = parser.parse_args()

sqWidth = 10 #number of squares width
sqHeight = 8 #number of squares height
allCorners = [] #all Charuco Corners
allIds = [] #all Charuco Ids
decimator = 0
#cameraMatrix = np.array([])
#disCoeffs = np.array([])

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(sqWidth,sqHeight,0.035,0.0175,dictionary)

cap = cv2.VideoCapture(0)

pictures = []

timer_start = time.time()

picture_taken = 10

# take pictures
while True:

    ret, frame = cap.read()

    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow("image", frame)
    key = cv2.waitKey(1)&0xFF

    if key == ord('q'):
        exit(1)

    if (time.time() - timer_start) > 2:
        pictures.append(frame)
        print("tic")
        timer_start = time.time()
        picture_taken -= 1

    if picture_taken == 0:
        break

print(len(pictures))

for img in pictures:

    if key == ord('q'):
        exit(1)

    [markerCorners,markerIds,rejectedImgPoints] = cv2.aruco.detectMarkers(img,dictionary)

    if len(markerCorners)>0:
        [ret,charucoCorners,charucoIds] = cv2.aruco.interpolateCornersCharuco(markerCorners,markerIds,img,board)
        if charucoCorners is not None and charucoIds is not None and len(charucoCorners)>3 and decimator%3==0:
            allCorners.append(charucoCorners)
            allIds.append(charucoIds)

        cv2.aruco.drawDetectedMarkers(img,markerCorners,markerIds)
        cv2.aruco.drawDetectedCornersCharuco(img,charucoCorners,charucoIds)

        #for corner in allCorners:
        #    cv2.circle(img,(corner[0][0], corner[0][0]),50,(255,255,255))

    cv2.imshow("frame",img)
    cv2.waitKey(0) #any key
    decimator+=1

imsize = img.shape
print(imsize)

cameraMatrixInit = np.array([[ 2000.,    0., imsize[0]/2.],
                                 [    0., 2000., imsize[1]/2.],
                                 [    0.,    0.,           1.]])

distCoeffsInit = np.zeros((5,1))
flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL)
(ret, camera_matrix, distortion_coefficients0,
    rotation_vectors, translation_vectors,
    stdDeviationsIntrinsics, stdDeviationsExtrinsics,
    perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
                      allCorners,
                      allIds,
                      board,
                      imsize,
                      cameraMatrixInit,
                      distCoeffsInit)


print("Press any key on window to exit")
cv2.waitKey(0) #any key
cv2.destroyAllWindows()