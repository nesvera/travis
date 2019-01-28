import numpy as np
import cv2
import sys
import os
import matplotlib.pyplot as plt

from homography import Homography

from cv_bridge import CvBridge

def nothing(data):
    pass

class FilterParameters:

    def __init__(self):

        self.gray_lower_bound = 95
        self.gray_upper_bound = 255

    def create_trackbar(self):

        cv2.namedWindow("filter", cv2.WINDOW_NORMAL)
        cv2.createTrackbar("gray_lower_bound", "filter", 0, 255, nothing)
        cv2.createTrackbar("gray_upper_bound", "filter", 0, 255, nothing)


    def update_trackbar_values(self):
        
        self.gray_lower_bound = cv2.getTrackbarPos("gray_lower_bound", "filter")
        self.gray_upper_bound = cv2.getTrackbarPos("gray_upper_bound", "filter")

    def set_trackbar_values(self):
        pass


class LaneDetector():

    def __init__(self, homography_file):

        self.bridge = CvBridge()

        self.homography_file = str(homography_file)
        self.homography = Homography(self.homography_file)

        self.homography_matrix = self.homography.get_homography_matrix()

        if self.homography_matrix is None:
            print("File doesnt have a homography matrix")
            print("Run find_homography script")

        self.filter_param = FilterParameters()
        #self.filter_param.create_trackbar()


        print("caiu")

        


    def filter(self, image):

        print(self.filter_param.gray_lower_bound, self.filter_param.gray_upper_bound)

        ret, thr = cv2.threshold(image, self.filter_param.gray_lower_bound, self.filter_param.gray_upper_bound, cv2.THRESH_BINARY)
        #ret, thr = cv2.threshold(image, self.filter_param.gray_lower_bound, self.filter_param.gray_upper_bound, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        filtered = thr

        return filtered

    def find_lane(self, image):
        
        (height, width) = image.shape

        new_image = image.copy()

        
        plt.ion()

        for l in range(height):
            line = image[l, :]

            new_image = cv2.line(new_image, (0,l), (width-1, l), (0,0,0), 10)
            cv2.imshow("aaa", new_image)
            cv2.waitKey(1)

            plt.plot(line)
            plt.draw()
            plt.pause(0.1)
            plt.clf()
            print("caiu")

        print("acabou")
        raw_input()
        


    def process(self, compressed_image):
        #self.filter_param.create_trackbar()
        #self.filter_param.update_trackbar_values()

        image = self.bridge.compressed_imgmsg_to_cv2(compressed_image, "bgr8")

        bird_view = cv2.warpPerspective(image.copy(), self.homography_matrix, (400, 400))
        gray = cv2.cvtColor(bird_view, cv2.COLOR_BGR2GRAY)  

        #self.filter_param.update_trackbar_values()

        filtered = self.filter(gray)

        self.find_lane(filtered)

        cv2.imshow("image", image)
        cv2.imshow("bird view", filtered)
        cv2.waitKey(1)