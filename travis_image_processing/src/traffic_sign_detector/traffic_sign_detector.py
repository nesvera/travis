import numpy as np
import cv2
from cv2 import aruco
import ArucoTaura as AT
import time

from travis_msg.msg import SignInfo
from travis_msg.msg import SignsDetected

def nothing(data):
    pass

class SignDetector():

    ID_NO_ENTRY     = 0
    ID_DEAD_END     = 1
    ID_RIGHT        = 2
    ID_LEFT         = 3
    ID_FORWARD      = 4
    ID_STOP         = 5

    '''
        Size in cm
    '''
    def __init__(self, size):

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_APRILTAG_36h11)
        self.aruco_parameters = aruco.DetectorParameters_create()

        self.marker_size = size

        self.vec = [np.array([[720.04174022,   0.        , 228.25281971],
        [  0.        , 718.79240652, 196.52969052],
        [  0.        ,   0.        ,   1.        ]]), np.array([[ 3.77911985e-02,  5.09255127e+00,  2.41566504e-02,
        -4.53035005e-02, -4.73635421e+01]])]
        
        self.arucos = AT.ArucoTaura(20, self.marker_size, self.vec)

    def debug(self):

        while True:

            try:

                cv2.imshow("image", self.aruco_draw)

                key = cv2.waitKey(1)&0xFF

                if key == ord('q'):
                    break

            except:
                pass

    def process(self, compressed_image):  

        self.arucos.feed(compressed_image)

        self.aruco_draw = self.arucos.drawAruco()

        sign_list = []

        for i in range(0, 6):
            distance = self.arucos.findDistance(i)

            if distance != False:

                sign = SignInfo()
                sign.id = i
                sign.distance = distance

                sign_list.append(sign)

        signs_detected = SignsDetected()
        signs_detected.sign_detected = sign_list

        print("--------------------------")

        print(signs_detected.sign_detected)

        return signs_detected