import numpy as np
import cv2
import sys
import os
import matplotlib.pyplot as plt
import time
import pickle
import random
import copy

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

    def debug(self):
    
        while True:
            try:
                #os.system('clear')

                #print("Numero de linhas: " + str(len(self.lanes_list)))
                for i, lane in enumerate(self.lanes_list):

                    distance = lane.distance_y()
                    #print("--> " + str(i) + " width: " + str(lane.get_avg_width()) + " len: " + str(lane.get_len_point()))


                    f = lane.get_function()

                    #for i in range(self.filter_param.roi_0_y, self.filter_param.roi_1_y):
                    for i in range(0, 399):
                        x = int(f(i))
                        y = i
                        cv2.circle(self.warp_res, (x, y), 3, 255, 1)

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
        self.color_res = cv2.cvtColor(self.image.copy(), cv2.COLOR_BGR2GRAY)
        self.warp_res = cv2.warpPerspective(self.color_res.copy(), self.homography_matrix, (400, 400))
        self.filter_res = self.filter(self.warp_res)
        self.lanes_list = self.find_lanes(self.filter_res.copy())

        # process found lanes
        lane_status = LaneInfo()
        img_h, img_w = self.warp_res.shape
        img_center_w = img_w/2.0

        # point to react
        y_react_point = self.filter_param.roi_1_y

        # delete small lanes
        for i, lane in enumerate(self.lanes_list):
            if lane.get_len_point() <= 2:
                del self.lanes_list[i]

        # 
        lane_left = None
        lane_center = None
        lane_right = None

        num_lanes = len(self.lanes_list)

        if num_lanes == 0:
            lane_status.lane_type = -1
            return lane_status
        
        # ONE LANE FOUND
        elif num_lanes == 1:
            lane = self.lanes_list[0]
            lane.distance_y()

            min_p = lane.get_min_vertical()
            max_p = lane.get_max_vertical()

            print(min_p, max_p)

            m_coef = (max_p[1]-min_p[1])/float((max_p[0]-min_p[0]))

            print("coef: " + str(m_coef))

            '''
            x_react_point = f(y_react_point)

            # left side, off the road
            if (img_center_w - x_react_point) >= 0:
                lane_status.lane_type = 0
                lane_status.lane_position = self.POS_OFF_ROAD_LEFT

                print("direeeeeeeeeeeeeeeeeeeeeeeita")
            
            else:
                lane_status.lane_type = 0
                lane_status.lane_position = self.POS_OFF_ROAD_RIGHT
                print("esqueeeeeeeeeeeeeeeeeeeerda")

            lane_status.lane_offset = -(img_center_w - x_react_point)
            #calcular curvatura
            '''
            
        # TWO LANES FOUND
        elif num_lanes == 2:
            # conferir distancia minima
            # conferir a distancia entre elas, se for pequeno
            # eh a do meio e a lateral
            # se for grande eh as duas laterais
            print("duuuuuuuuuuuuuuas")
            pass

        # THREE LANES FOUND
        elif num_lanes == 3:
            print("tresssssssssssssss")
            pass

        else:
            print("fudeeeeeeeeeeee")
            pass


        # find line with min, max width
        max_width = self.lanes_list[0].get_avg_width()
        max_width_index = 0

        min_width = self.lanes_list[0].get_avg_width()

        for i, lane in enumerate(self.lanes_list):
            if lane.get_avg_width() > max_width:
                max_width = lane.get_avg_width()
                max_width_index = i

            if lane.get_avg_width() < min_width:
                min_width = lane.get_avg_width()

        # detect 1 or 2 way road
        if (max_width - min_width) < 10:
            lane_status.lane_type = 1

        else:
            lane_status.lane_type = 2

        lane_status.lane_position = 0
        lane_status.lane_offset = 0
        lane_status.lane_curvature = 0

        periodo = time.time() - start
        fps = 1/periodo
        print("FPS: " + str(fps))

        return lane_status

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

        self.search_box_w = 20
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