import numpy as np
import cv2
import sys
import os
import matplotlib.pyplot as plt
import time

from homography import Homography

from cv_bridge import CvBridge

def nothing(data):
    pass

class Parameters:

    def __init__(self):

        # black/white markers
        self.gray_lower_bound = 95
        self.gray_upper_bound = 255

        # colored markers

        #
        self.min_width = 0
        self.max_width = 0

        self.orientation = 0

        self.roi_0_x = 0
        self.roi_0_y = 0
        self.roi_1_x = 0
        self.roi_1_y = 0


    def create_trackbar(self):

        cv2.namedWindow("filter", cv2.WINDOW_NORMAL)
        cv2.createTrackbar("gray_lower_bound", "filter", 0, 255, nothing)
        cv2.createTrackbar("gray_upper_bound", "filter", 0, 255, nothing)


    def update_trackbar_values(self):
        
        self.gray_lower_bound = cv2.getTrackbarPos("gray_lower_bound", "filter")
        self.gray_upper_bound = cv2.getTrackbarPos("gray_upper_bound", "filter")

    def load_parameters(self, file):
        pass

    def save_parameters(self, obj, file):
        pass


class Lane:

    '''
        point = np.array of pixel coordinate
        width = number bright pixels
    '''
    def __init__(self, point, width):

        self.points_list = list(point)
        self.width_list = list(width)

    def add_point(point, width):

        self.points_list.append(point)
        self.width_list.append(width)

    def get_last_point():

        return self.points_list.append[-1]


class LaneDetector():

    def __init__(self, homography_file, filter_file, tune_param ):

        self.bridge = CvBridge()

        self.homography_file = str(homography_file)
        self.homography = Homography(self.homography_file)

        self.homography_matrix = self.homography.get_homography_matrix()

        if self.homography_matrix is None:
            print("File doesnt have a homography matrix")
            print("Run find_homography script")

        self.filter_param = Parameters()

        self.tune_param = tune_param
        if tune_param:

            self.filter_param.create_trackbar()

        else:

            pass

        self.init = True
        
    def process(self, compressed_image):
        #self.filter_param.create_trackbar()
        #self.filter_param.update_trackbar_values()

        #image = self.bridge.compressed_imgmsg_to_cv2(compressed_image, "bgr8")
        image = compressed_image

        #if self.init == True:
        #    cv2.imwrite("/home/nesvera/image_test.jpg", image)

        bird_view = cv2.warpPerspective(image.copy(), self.homography_matrix, (400, 400))
        gray = cv2.cvtColor(bird_view, cv2.COLOR_BGR2GRAY)  

        #filtered = self.filter(gray)

        #self.histogram_view(filtered)
        #self.find_lane(filtered)

        if self.tune_param == 1:

            cv2.imshow("image", image)
            #cv2.imshow("gray", gray)
            #cv2.imshow("bird view", filtered)

            key = cv2.waitKey(1)&0xFF

            if key == ord('q'):
                cv2.destroyAllWindows()
                exit(1)

            elif key == ord('l'):
                self.filter_param.load_file()

            elif key == ord('s'):
                self.filter_param.save_file()


            self.filter_param.update_trackbar_values()