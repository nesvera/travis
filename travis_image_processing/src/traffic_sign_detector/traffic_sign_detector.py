import numpy as np
import cv2
from cv2 import aruco

def nothing(data):
    pass

class SignDetector():

    '''
        Size in cm
    '''
    def __init__(self, size, calibrate):

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)
        self.aruco_parameters = aruco.DetectorParameters_create()

        self.calibrate = calibrate

        self.marker_size = size

        if self.calibrate == True:  
            pass
        
    def process(self, compressed_image):

        if self.calibrate == True:

            gray = cv2.cvtColor(compressed_image, cv2.COLOR_BGR2GRAY)

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray,
                                                                self.aruco_dict,
                                                                parameters=self.aruco_parameters)
                                                            
            frame_markers = aruco.drawDetectedMarkers(compressed_image.copy(),
                                                    corners,
                                                    ids)     

            know_distance = 30  #cm

            if len(corners) > 0:
                c1 = corners[0][0][0]
                c2 = corners[0][0][1]

                side_size_pixel = np.linalg.norm(c2-c1)

                #print(c1, c2)
                #print(side_size_pixel)

                cv2.circle(compressed_image, (c1[0], c1[1]), 5, (255,0,0), 2)
                cv2.circle(compressed_image, (c2[0], c2[1]), 5, (255,255,0), 2)

                #focal_length = (side_size_pixel*know_distance)/self.marker_size
                focal_length = 720

                distance = (self.marker_size*focal_length)/side_size_pixel

                print(focal_length, distance)


            cv2.imshow("image", compressed_image)
            #cv2.imshow("aruco", frame_markers)

            key = cv2.waitKey(1)&0xFF

            if key == ord('q'):
                exit(1)

        else:
            pass

    #def get_distance(self.marker_size, )