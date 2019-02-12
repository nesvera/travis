import numpy as np
import cv2
import sys
import os
import matplotlib.pyplot as plt
import time
import pickle

from homography import Homography

from cv_bridge import CvBridge

def nothing(data):
    pass

class Lane:

    '''
        point = np.array of pixel coordinate
        width = number bright pixels
    '''
    def __init__(self, point, width):

        self.points_list = list([point])
        self.width_list = list(width)

    def add_point(point, width):

        self.points_list.append(point)
        self.width_list.append(width)

    def get_last_point():

        return self.points_list.append[-1]


class LaneDetector():

    def __init__(self, homography_file, filter_file, tune_param ):

        self.bridge = CvBridge()

        # load homography matrix
        self.homography_file = str(homography_file)
        self.homography = Homography(self.homography_file)

        self.homography_matrix = self.homography.get_homography_matrix()

        if self.homography_matrix is None:
            print("File doesnt have a homography matrix")
            print("Run find_homography script")

        # load parameters used in the filter process
        self.filter_file = filter_file
        self.filter_param = Parameters(filter_file)
        self.tune_param = tune_param

        # configure parameters
        if tune_param:
            self.filter_param.create_trackbar()
            self.filter_param.load()
            self.filter_param.set_trackbar_values()

        # autonomous mode
        else:
            self.filter_param.load()

            pass

        self.init = True

        # 
        self.filter_res = 0
        
    def process(self, compressed_image):
        #self.filter_param.create_trackbar()
        #self.filter_param.update_trackbar_values()

        #image = self.bridge.compressed_imgmsg_to_cv2(compressed_image, "bgr8")
        image = compressed_image

        #if self.init == True:
        #    cv2.imwrite("/home/nesvera/image_test.jpg", image)

        warp_res = cv2.warpPerspective(image.copy(), self.homography_matrix, (400, 400))
        
        #color_res = warp_res
        color_res = cv2.cvtColor(warp_res, cv2.COLOR_BGR2GRAY)  

        filter_res = self.filter(color_res)

        self.find_lanes(filter_res)

        # configure parameters mode
        if self.tune_param == 1:

            cv2.imshow("image", image)
            cv2.imshow("warp_res", warp_res)
            cv2.imshow("color_res", color_res)
            cv2.imshow("filter_res", filter_res)

            key = cv2.waitKey(1)&0xFF

            if key == ord('q'):
                cv2.destroyAllWindows()
                exit(1)

            elif key == ord('l'):
                self.filter_param.load()
                self.filter_param.set_trackbar_values()

            elif key == ord('s'):
                self.filter_param.save(self.filter_param)

            self.filter_param.update_trackbar_values()

    def filter(self, image):

        ret, thr = cv2.threshold(image, self.filter_param.gray_lower_bound, self.filter_param.gray_upper_bound, cv2.THRESH_BINARY)
        #ret, thr = cv2.threshold(image, self.filter_param.gray_lower_bound, self.filter_param.gray_upper_bound, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        # talvez aplicar sobel(gradient) + threshould

        filtered = thr

        return filtered

    def find_lanes(self, image):

        cv2.rectangle(image, 
                      (self.filter_param.roi_0_x, self.filter_param.roi_0_y),
                      (self.filter_param.roi_1_x, self.filter_param.roi_1_y),
                      255, 10)

        cv2.imshow("rect", image)

class Parameters:

    def __init__(self, file):

        # black/white markers
        self.gray_lower_bound = 95
        self.gray_upper_bound = 255

        # colored markers
        self.h_min = 0
        self.h_max = 0
        self.s_min = 0
        self.s_max = 0
        self.v_min = 0
        self.v_max = 0

        #
        self.lane_min_width = 0
        self.lane_max_width = 0

        self.roi_0_x = 0
        self.roi_0_y = 0
        self.roi_1_x = 0
        self.roi_1_y = 0

        self.file = file


    def create_trackbar(self):

        cv2.namedWindow("filter", cv2.WINDOW_NORMAL)
        cv2.createTrackbar("gray_lower_bound", "filter", 0, 255, nothing)
        cv2.createTrackbar("gray_upper_bound", "filter", 0, 255, nothing)
        cv2.createTrackbar("h_min", "filter", 0, 255, nothing)
        cv2.createTrackbar("h_max", "filter", 0, 255, nothing)
        cv2.createTrackbar("s_min", "filter", 0, 255, nothing)
        cv2.createTrackbar("s_max", "filter", 0, 255, nothing)
        cv2.createTrackbar("v_min", "filter", 0, 255, nothing)
        cv2.createTrackbar("v_max", "filter", 0, 255, nothing)
        cv2.createTrackbar("lane_min_width", "filter", 0, 720, nothing)
        cv2.createTrackbar("lane_max_width", "filter", 0, 720, nothing)
        cv2.createTrackbar("roi_0_x", "filter", 0, 720, nothing)
        cv2.createTrackbar("roi_0_y", "filter", 0, 720, nothing)
        cv2.createTrackbar("roi_1_x", "filter", 0, 720, nothing)
        cv2.createTrackbar("roi_1_y", "filter", 0, 720, nothing)

    def update_trackbar_values(self):
        
        self.gray_lower_bound = cv2.getTrackbarPos("gray_lower_bound", "filter")
        self.gray_upper_bound = cv2.getTrackbarPos("gray_upper_bound", "filter")
        self.h_min = cv2.getTrackbarPos("h_min", "filter")
        self.h_max = cv2.getTrackbarPos("h_max", "filter")
        self.s_min = cv2.getTrackbarPos("s_min", "filter")
        self.s_max = cv2.getTrackbarPos("s_max", "filter")
        self.v_min = cv2.getTrackbarPos("v_min", "filter")
        self.v_max = cv2.getTrackbarPos("v_max", "filter")
        self.lane_min_width = cv2.getTrackbarPos("lane_min_width", "filter")
        self.lane_max_width = cv2.getTrackbarPos("lane_max_width", "filter")
        self.roi_0_x = cv2.getTrackbarPos("roi_0_x", "filter")
        self.roi_0_y = cv2.getTrackbarPos("roi_0_y", "filter")
        self.roi_1_x = cv2.getTrackbarPos("roi_1_x", "filter")
        self.roi_1_y = cv2.getTrackbarPos("roi_1_y", "filter")

    def set_trackbar_values(self):

        cv2.setTrackbarPos("gray_lower_bound", "filter", self.gray_lower_bound)
        cv2.setTrackbarPos("gray_upper_bound", "filter", self.gray_upper_bound)
        cv2.setTrackbarPos("h_min", "filter", self.h_min)
        cv2.setTrackbarPos("h_max", "filter", self.h_max)
        cv2.setTrackbarPos("s_min", "filter", self.s_min)
        cv2.setTrackbarPos("s_max", "filter", self.s_max)
        cv2.setTrackbarPos("v_min", "filter", self.v_min)
        cv2.setTrackbarPos("v_max", "filter", self.v_max)
        cv2.setTrackbarPos("lane_min_width", "filter", self.lane_min_width)
        cv2.setTrackbarPos("lane_max_width", "filter", self.lane_max_width)
        cv2.setTrackbarPos("roi_0_x", "filter", self.roi_0_x)
        cv2.setTrackbarPos("roi_0_y", "filter", self.roi_0_y)
        cv2.setTrackbarPos("roi_1_x", "filter", self.roi_1_x)
        cv2.setTrackbarPos("roi_1_y", "filter", self.roi_1_y)

    def load(self):
        
        file_obj = open(self.file, 'r')

        try:
            param_dict = pickle.load(file_obj)
        except:
            print("Failed to open pickle file!")
            exit(1)

        file_obj.close()

        self.gray_lower_bound = param_dict['gray_lower_bound']
        self.gray_upper_bound = param_dict['gray_upper_bound']
        self.h_min = param_dict['h_min']
        self.h_max = param_dict['h_max']
        self.s_min = param_dict['s_min']
        self.s_max = param_dict['s_max']
        self.v_min = param_dict['v_min']
        self.v_max = param_dict['v_max']
        self.lane_min_width = param_dict['lane_min_width']
        self.lane_max_width = param_dict['lane_max_width']
        self.roi_0_x = param_dict['roi_0_x']
        self.roi_0_y = param_dict['roi_0_y']
        self.roi_1_x = param_dict['roi_1_x']
        self.roi_1_y = param_dict['roi_1_y']

    def save(self, obj):
        
        file_obj = open(self.file, 'w')
        class_to_dict = obj.__dict__
        pickle.dump(class_to_dict, file_obj, pickle.HIGHEST_PROTOCOL)
        file_obj.close()