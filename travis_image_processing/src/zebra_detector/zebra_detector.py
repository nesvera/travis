import numpy as np
import cv2
import sys
import os
import matplotlib.pyplot as plt
import time
import pickle
import random

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
        self.width_list = list([width])

        self.curve_coef = None
        self.curve_f = None

    def add_point(self, point, width):

        self.points_list.append(point)
        self.width_list.append(width)

    def get_last_point(self):

        return self.points_list[-1]

    def get_points(self):

        return self.points_list

    def fit_curve(self):

        np_points = np.array(self.points_list)

        x = np_points[:, 0]
        y = np_points[:, 1]

        self.curve_coef = np.polyfit(y, x, 2)
        self.curve_f = np.poly1d(self.curve_coef)

    def get_function(self):

        self.fit_curve()
        return self.curve_f

    def distance_x(self):

        pass

    def distance_y(self):

        # initialize variabels
        min_point = self.points_list[0]
        max_point = self.points_list[0]

        for point in self.points_list:

            if point[1] < min_point[1]:
                min_point = point

            if point[1] > max_point[1]:
                max_point = point
        
        
        return min_point, max_point



class ZebraDetector():

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

            if os.path.exists(self.filter_file):
                self.filter_param.load()
                self.filter_param.set_trackbar_values()

        # autonomous mode
        else:
            if os.path.exists(self.filter_file):
                self.filter_param.load()

            else:
                print("Filter file doesnt exist")
                exit(1)

        self.init = True

        # 
        self.filter_res = 0
        
    def process(self, compressed_image):
        #self.filter_param.create_trackbar()
        #self.filter_param.update_trackbar_values()

        start = time.time()

        #image = self.bridge.compressed_imgmsg_to_cv2(compressed_image, "bgr8")
        image = compressed_image

        #if self.init == True:
        #    cv2.imwrite("/home/nesvera/image_test.jpg", image)

        warp_res = cv2.warpPerspective(image.copy(), self.homography_matrix, (400, 400))
        
        #color_res = warp_res
        color_res = cv2.cvtColor(warp_res, cv2.COLOR_BGR2GRAY)  

        filter_res = self.filter(color_res)

        self.find_lanes(filter_res.copy())

        periodo = time.time() - start
        fps = 1/periodo

        print(fps)

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

        ret, thr = cv2.threshold(image, self.filter_param.gray_lower_bound, 255, cv2.THRESH_BINARY)
        #ret, thr = cv2.threshold(image, self.filter_param.gray_lower_bound, self.filter_param.gray_upper_bound, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        # talvez aplicar sobel(gradient) + threshould

        filtered = thr

        return filtered

    def find_lanes(self, image):

        '''
            top-left = roi_0_x, roi_0_y
            bot-right = roi_1_x, roi_1_y


        '''
        # image size
        img_h, img_w = image.shape

        # initial search position y
        #y_pos_init = self.filter_param.roi_0_y
        
        # random initial position of y
        #y_pos_init = random.randint(self.filter_param.roi_0_y, 
        #                            self.filter_param.roi_1_y + 1)

        y_pos_init = int((self.filter_param.roi_0_y + self.filter_param.roi_1_y)/2)

        # get entire horizontal line for the first scan of lanes
        hor_line = image[y_pos_init, :]

        # list of lanes... lane is a class with points, width ... 
        lanes_list = []

        # store temporary points for a interesting area of the horizontal search
        points_found = []

        for index, pixel in enumerate(hor_line):

            if pixel == 255:
                points_found.append(index)

            elif pixel == 0 and len(points_found) > 0:

                # after to collect all sequence of bright pixels, get the median
                new_lane_center_point = np.array((int(np.median(points_found)), y_pos_init))
                points_found = []

                # create a Lane obj and add to the list of lanes
                lanes_list.append( Lane(new_lane_center_point, len(points_found)) )

        # from the points found in the first search, find all points connected to this lane
        # using sliding window with a box of fixed size
        for lane in lanes_list:

            # get last point found of a lane
            last_point = lane.get_last_point()
            x_lane_start = last_point[0]
            y_lane_start = last_point[1]

            # search up
            x_pos_cur = x_lane_start
            y_pos_cur = y_lane_start + 1

            points_found = []

            while y_pos_cur >= self.filter_param.roi_0_y:

                x_box_start = x_pos_cur-int(self.filter_param.search_box_w/2)
                x_box_end = x_pos_cur+int(self.filter_param.search_box_w/2)

                # get pixels from a search box
                hor_line = image[ y_pos_cur, x_box_start:x_box_end]

                # real index of a hor_line pixel in the image
                hor_line_image_index = [i for i in range(x_box_start,x_box_end)]

                for index, pixel in enumerate(hor_line):

                    # store bright pixels
                    if pixel == 255:
                        points_found.append(index)

                if len(points_found) > 0:

                    lane_center = hor_line_image_index[int(np.median(points_found))]

                    new_lane_point = np.array((lane_center, y_pos_cur))
                    points_found = []
                
                    # append a new point to the list
                    lane.add_point(new_lane_point, len(points_found))

                    # x start point for the next search
                    x_pos_cur = lane_center

                    # "feature" existente aqui: como nao tem uma condicao de parada
                    # quando a linha acabar a baixa continuara procurando seguindo verticalmente
                    # no ultimo x. Talvez seja bom para a pista com faixa pontilhada, mas talvez
                    # de problema....
                    # solucao: caso nao ter nenhum ponto brilhante, subir n posicoes, se nao houver
                    # um ponto brilhante, acaba

                # move window to the next line
                y_pos_cur = y_pos_cur - 1
                
            # search down
            x_pos_cur = x_lane_start
            y_pos_cur = y_lane_start + 1

            points_found = []

            while y_pos_cur <= self.filter_param.roi_1_y:

                x_box_start = x_pos_cur-int(self.filter_param.search_box_w/2)
                x_box_end = x_pos_cur+int(self.filter_param.search_box_w/2)

                # get pixels from a search box
                hor_line = image[ y_pos_cur, x_box_start:x_box_end]

                # real index of a hor_line pixel in the image
                hor_line_image_index = [i for i in range(x_box_start,x_box_end)]

                for index, pixel in enumerate(hor_line):

                    # store bright pixels
                    if pixel == 255:
                        points_found.append(index)

                if len(points_found) > 0:

                    lane_center = hor_line_image_index[int(np.median(points_found))]

                    new_lane_point = np.array((lane_center, y_pos_cur))
                    points_found = []
                
                    # append a new point to the list
                    lane.add_point(new_lane_point, len(points_found))

                    # x start point for the next search
                    x_pos_cur = lane_center

                    # "feature" existente aqui: como nao tem uma condicao de parada
                    # quando a linha acabar a baixa continuara procurando seguindo verticalmente
                    # no ultimo x. Talvez seja bom para a pista com faixa pontilhada, mas talvez
                    # de problema....
                    # solucao: caso nao ter nenhum ponto brilhante, subir n posicoes, se nao houver
                    # um ponto brilhante, acaba

                # move window to the next line
                y_pos_cur = y_pos_cur + 1

        print(len(lanes_list))

        '''
        for lane in lanes_list:

            min_p, max_p = lane.distance_y()
            f = lane.get_function()

            #for i in range(self.filter_param.roi_0_y, self.filter_param.roi_1_y):
            for i in range(0, 399):
                x = int(f(i))
                y = i
                cv2.circle(image, (x, y), 3, 255, 1)
        '''

        cv2.rectangle(image, 
                      (self.filter_param.roi_0_x, self.filter_param.roi_0_y),
                      (self.filter_param.roi_1_x, self.filter_param.roi_1_y),
                      255, 2)

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

        self.search_box_w = 8
        self.search_box_h = 8

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