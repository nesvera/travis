import numpy as np
import cv2
import sys
import os
import matplotlib.pyplot as plt
import time
import pickle
import random
import copy
from collections import deque

from homography import Homography

from cv_bridge import CvBridge

from travis_msg.msg import LaneInfo

def nothing(data):
    pass

class Lane:

    '''
        point = np.array of pixel coordinate
        width = number bright pixels
    '''
    def __init__(self, point, width):

        self.points_list = list([point])
        self.width_sum = width

        self.curve_coef = None
        self.curve_f = None

        self.min_hor = (0,0)
        self.max_hor = (0,0)
        self.min_ver = (0,0)
        self.max_ver = (0,0)

    def add_point(self, point, width):

        self.points_list.append(point)
        self.width_sum += width

    def get_last_point(self):

        return self.points_list[-1]

    def get_points(self):

        return self.points_list
    
    def get_avg_width(self):

        return self.width_sum

    def get_len_point(self):

        return len(self.points_list)

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
        
        self.min_ver = min_point
        self.max_ver = max_point

        return np.linalg.norm(max_point-min_point)

    def get_min_vertical(self):
        return self.min_ver

    def get_max_vertical(self):
        return self.max_ver

class LaneDetector():

    POS_OFF_ROAD_RIGHT = 0
    POS_ON_ROAD_RIGHT = 1
    POS_ON_ROAD_LEFT = 2
    POS_OFF_ROAD_LEFT = 3

    ROAD_NOT_FOUND = 0
    ROAD_UNKOWN = 1
    ROAD_1_WAY = 2
    ROAD_2_WAY = 3

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

        self.image = None
        self.warp_res = None
        self.color_res = None
        self.filter_res = None

        self.lanes_list = None

        self.fps_buffer = deque(maxlen=10)

    def debug(self):
    
        while True:
            try:
                #os.system('clear')

                #print("Numero de linhas: " + str(len(self.lanes_list)))
                for i, lane in enumerate(self.lanes_list):

                    distance = lane.distance_y()
                    #print("--> " + str(i) + " width: " + str(lane.get_avg_width()) + " len: " + str(lane.get_len_point()))


                    f = lane.get_function()

                    #for j in range(self.filter_param.roi_0_y, self.filter_param.roi_1_y):
                    #for j in range(0, 399):
                    #    x = int(f(j))
                    #    y = j
                    #    cv2.circle(self.warp_res, (x, y), 3, 0, 1)

                    points = lane.get_points()

                    for p in points:
                        cv2.circle(self.warp_res, (p[0], p[1]), 3, 0, 1)


                cv2.rectangle(self.warp_res, 
                                (self.filter_param.roi_0_x, self.filter_param.roi_0_y),
                                (self.filter_param.roi_1_x, self.filter_param.roi_1_y),
                                255, 2)

                cv2.imshow("image", self.image)
                cv2.imshow("warp_res", self.warp_res)
                cv2.imshow("color_res", self.color_res)
                cv2.imshow("filter_res", self.filter_res)

                key = cv2.waitKey(1)&0xFF

                if key == ord('q'):
                    cv2.destroyAllWindows()
                    break

                elif key == ord('l'):
                    self.filter_param.load()
                    self.filter_param.set_trackbar_values()

                elif key == ord('s'):
                    self.filter_param.save(self.filter_param)

                self.filter_param.update_trackbar_values()

            except:
                pass
        
    def process(self, compressed_image):

        start = time.time()

        self.image = self.bridge.compressed_imgmsg_to_cv2(compressed_image, "bgr8")
        #self.image = compressed_image.copy()
        self.color_res = cv2.cvtColor(self.image.copy(), cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        contrast = clahe.apply(self.color_res)

        self.warp_res = cv2.warpPerspective(contrast, self.homography_matrix, (400, 400))
        self.filter_res = self.filter(self.warp_res)
        self.lanes_list = self.find_lanes(self.filter_res.copy())   

        good_lanes = []

        # delete small lanes
        for lane in self.lanes_list:

            if lane.get_len_point() > 10:
                good_lanes.append(lane)

        self.lanes_list = good_lanes

        # classify the road
        lane_status = LaneInfo()
        img_h, img_w = self.warp_res.shape
        img_w_center = img_w/2.0
        y_react_point = self.filter_param.roi_1_y

        num_lanes = len(self.lanes_list)
        if num_lanes == 0:
            lane_status.lane_type = self.ROAD_NOT_FOUND
            return lane_status

        elif num_lanes == 1:
            lane_status.lane_type = self.ROAD_UNKOWN

            lane = self.lanes_list[0]
            lane.distance_y()

            max_p = lane.get_min_vertical()
            min_p = lane.get_max_vertical()

            #print(min_p, max_p)

            m_coef = -(min_p[1]-max_p[1])/float((min_p[0]-max_p[0]))
            angle = np.arctan(m_coef)*180/np.pi

            #print(min_p, max_p)
            print("angle: " + str(angle))
            #raw_input()

            x_react_point = min_p[0]
            y_react_point = max_p[1]

            #print(x_react_point, y_react_point)
            #print(img_w_center-x_react_point)

            # left side, off the road
            if angle <= 85 and angle >= 0:
               
                lane_status.lane_type = 0

                s_angle = 90 - angle
                s_offset = 80 - (img_w_center-x_react_point)
                
                lane_status.lane_curvature = s_angle
                lane_status.lane_offset = s_angle + s_offset

                print(lane_status.lane_offset)

                print("direeeeeeeeeeeeeeeeeeeeeeeita")
            
            elif  angle >= -85 and angle <= 0:
                lane_status.lane_type = 0

                s_angle = -(90 + angle)
                s_offset = -(60 + (img_w_center-x_react_point))

                lane_status.lane_curvature = s_angle
                lane_status.lane_offset = s_angle + s_offset

                print(lane_status.lane_offset)

                print("esqueeeeeeeeeeeeeeeeeeeerda")

            else:
                lane_status.lane_type = 0
                lane_status.lane_curvature = 0
                lane_status.lane_offset = 0

                print("reeeeeto")
                pass

        elif num_lanes == 2:
            lane_status.lane_type = 1

            angle_list = []
            x_lane_position = 0

            for lane in self.lanes_list:
                lane.distance_y()

                f = lane.get_function()
                x_lane_position += f(self.filter_param.roi_1_y)

                max_p = lane.get_min_vertical()
                min_p = lane.get_max_vertical()

                #print(min_p, max_p)

                m_coef = -(min_p[1]-max_p[1])/float((min_p[0]-max_p[0]))
                angle = np.arctan(m_coef)*180/np.pi

                angle_list.append(abs(angle))

            lane_status.lane_curvature = min(angle_list)
            lane_status.lane_offset = ((x_lane_position)/2.0) - img_w_center

            print(lane_status.lane_offset)

        elif num_lanes == 3:
            lane_status.lane_type = 2

            good_lanes = []

            for i in range(len(self.lanes_list)-1):

                f1 = self.lanes_list[i].get_function()
                f2 = self.lanes_list[i+1].get_function()

                if abs(f1(self.filter_param.roi_1_y) - f2(self.filter_param.roi_1_y)) > 10:
                    good_lanes.append(self.lanes_list[i])

            good_lanes.append(self.lanes_list[-1])  

            angle_list = []
            x_lane_position = 0

            for lane in good_lanes:
                lane.distance_y()

                f = lane.get_function()
                x_lane_position += f(self.filter_param.roi_1_y)

                max_p = lane.get_min_vertical()
                min_p = lane.get_max_vertical()

                #print(min_p, max_p)

                m_coef = -(min_p[1]-max_p[1])/float((min_p[0]-max_p[0]))
                angle = np.arctan(m_coef)*180/np.pi

                angle_list.append(abs(angle))

            lane_status.lane_curvature = min(angle_list)
            lane_status.lane_offset = ((x_lane_position)/2.0) - img_w_center

            print(lane_status.lane_offset)                  


        elif num_lanes == 4:
            lane_status.lane_type = 2
                        

        periodo = time.time() - start
        fps = 1/periodo

        self.fps_buffer.append(fps)

        sum_fps = sum(self.fps_buffer)/len(self.fps_buffer)

        print("FPS: " + str(sum_fps))


        return lane_status

            

    def filter(self, image):

        ret, thr = cv2.threshold(image, self.filter_param.gray_lower_bound, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        filtered = cv2.morphologyEx(thr, cv2.MORPH_OPEN, kernel)

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
        y_pos_init = random.randint(self.filter_param.roi_0_y, 
                                    self.filter_param.roi_1_y + 1)

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

                # create a Lane obj and add to the list of lanes
                lanes_list.append( Lane(new_lane_center_point, len(points_found)) )

                points_found = []

        # from the points found in the first search, find all points connected to this lane
        # using sliding window with a box of fixed size
        for lane in lanes_list:

            # get last point found of a lane
            last_point = lane.get_last_point()
            x_lane_start = last_point[0]
            y_lane_start = last_point[1]

            # search up
            x_pos_cur = x_lane_start
            y_pos_cur = y_lane_start - self.filter_param.search_box_h

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
                
                    # append a new point to the list
                    lane.add_point(new_lane_point, len(points_found))

                    # x start point for the next search
                    x_pos_cur = lane_center

                    points_found = []

                    # "feature" existente aqui: como nao tem uma condicao de parada
                    # quando a linha acabar a baixa continuara procurando seguindo verticalmente
                    # no ultimo x. Talvez seja bom para a pista com faixa pontilhada, mas talvez
                    # de problema....
                    # solucao: caso nao ter nenhum ponto brilhante, subir n posicoes, se nao houver
                    # um ponto brilhante, acaba

                # if not was found following the lane
                else:
                    break

                # move window to the next line
                y_pos_cur = y_pos_cur - self.filter_param.search_box_h
                
            # search down
            x_pos_cur = x_lane_start
            y_pos_cur = y_lane_start + self.filter_param.search_box_h

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
                
                    # append a new point to the list
                    lane.add_point(new_lane_point, len(points_found))

                    # x start point for the next search
                    x_pos_cur = lane_center

                    points_found = []

                    # "feature" existente aqui: como nao tem uma condicao de parada
                    # quando a linha acabar a baixa continuara procurando seguindo verticalmente
                    # no ultimo x. Talvez seja bom para a pista com faixa pontilhada, mas talvez
                    # de problema....
                    # solucao: caso nao ter nenhum ponto brilhante, subir n posicoes, se nao houver
                    # um ponto brilhante, acaba

                # if not was found following the lane
                else:
                    break

                # move window to the next line
                y_pos_cur = y_pos_cur + self.filter_param.search_box_h

        return lanes_list


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

        self.search_box_w = 50
        self.search_box_h = 10

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

        cv2.namedWindow("filter", cv2.WINDOW_NORMAL)
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


'''
p2 = np.polyder(f)
            p3 = np.polyder(p2)

            point = (self.filter_param.roi_1_y+self.filter_param.roi_0_y)/2.0

            radius = ((1+(p2(point))**2)**(3/2.))/abs(p3(point))

            #for j in range(self.filter_param.roi_0_y, self.filter_param.roi_1_y):
            #    x = int(f(j))
            #    y = j
            #    cv2.circle(self.warp_res, (x, y), 3, 255, 1)

            points = lane.get_points()

            for p in points:
                cv2.circle(self.warp_res, (p[0], p[1]), 3, 0, 1)    
                pass

            cv2.rectangle(self.warp_res, 
                            (self.filter_param.roi_0_x, self.filter_param.roi_0_y),
                            (self.filter_param.roi_1_x, self.filter_param.roi_1_y),
                            255, 2)

            p1 = np.array((f(self.filter_param.roi_1_y), 
                          (self.filter_param.roi_1_y)))
            
            mid_y = random.randint(self.filter_param.roi_0_y, 
                                    self.filter_param.roi_1_y + 1)
            p2 = np.array((f((mid_y)/2), 
                  ((mid_y)/2)))
            
            p3 = np.array((f(self.filter_param.roi_0_y), 
                          (self.filter_param.roi_0_y)))

            #cv2.circle(self.warp_res, p2, 3, 0, 1) 
            d_x1 = (p2[1]-p1[1])
            d_y1 = float(p2[0]-p1[0]) if float(p2[0]-p1[0]) != 0 else 0.0000001 
            m1 = d_x1/d_y1

            d_x2 = (p3[1]-p2[1])
            d_y2 = float(p3[0]-p2[0]) if float(p3[0]-p2[0]) != 0 else 0.0000001 
            m2 = d_x2/d_y2

            dx_dy = (m1+m2)/2.0

            x_mid1 = (p2[0]+p1[0])/2.0
            x_mid2 = (p3[0]+p2[0])/2.0

            delta_x_mid = float(x_mid1-x_mid2) if float(x_mid1-x_mid2) != 0 else 0.0000001

            dx2_dy2 = (m2-m1)/delta_x_mid

            #adius = ((1+(dx_dy)**2)**(3/2.0))/abs(dx2_dy2)

            #print(p1, p2, p3)
            #print(m1, m2)
            print(radius)
'''