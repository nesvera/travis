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

    def filter(self, image):

        ret, thr = cv2.threshold(image, self.filter_param.gray_lower_bound, self.filter_param.gray_upper_bound, cv2.THRESH_BINARY)
        #ret, thr = cv2.threshold(image, self.filter_param.gray_lower_bound, self.filter_param.gray_upper_bound, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        # talvez aplicar sobel(gradient) + threshould

        filtered = thr

        return filtered

    def follow_lanes(self, image, img_width, img_height, y_pos_init, y_pos_end):

        # get entire horizontal line for the first scan of lanes
        hor_line = image[y_pos_init, :]

        # store tempory points for a interesting area of the horizontal search
        points_found = []

        # list of lanes, which list is composed of a list with points for the lane
        lanes_list = []

        for index, pixel in enumerate(hor_line):

            if pixel == 255:
                points_found.append(index)

            if pixel == 0  and len(points_found) > 0:
                
                # after to save of all points of an area of instest, get the median 
                new_lane_center_point = np.array((int(np.median(points_found)), y_pos_init))
                points_found = []

                lanes_list.append(list([new_lane_center_point]))

        # from the points found in the first search, find all points of a lane 
        # using sliding window with a box of fix size
        for lane in lanes_list:

            if y_pos_init < y_pos_end:
                y_pos_cur = y_pos_init + 1     

            else:
                y_pos_cur = y_pos_init - 1
            
            x_pos_cur = lane[-1][0]

            while True:

                hor_line = image[y_pos_cur, 
                                x_pos_cur-5:x_pos_cur+5]

                hor_line_positions = [i for i in range(x_pos_cur-4,x_pos_cur+4)]


                for index, pixel in enumerate(hor_line):

                    if pixel == 255:
                        points_found.append(index)

                if len(points_found) > 0:

                    lane_center = hor_line_positions[int(np.median(points_found))]
                
                    # after to save of all points of an area of instest, get the median 
                    new_lane_center_point = np.array((lane_center, y_pos_init))
                    points_found = []

                    lane.append(new_lane_center_point)

                    x_pos_cur = int(lane_center)
                    #print(lane_center)

                    #image = cv2.line(image, (x_pos_cur-5, y_pos_cur), (x_pos_cur+5, y_pos_cur), 255, 1)
                    #cv2.imshow("i", image)
                    #cv2.waitKey(1)

                # move window to the next line
                if y_pos_init < y_pos_end:
                    y_pos_cur = y_pos_cur + 1     

                else:
                    y_pos_cur = y_pos_cur - 1

                if y_pos_cur == y_pos_end:
                    break


    def follow_lanes_both_sides(self, image, img_width, img_height, y_pos_init):

        # get entire horizontal line for the first scan of lanes
        hor_line = image[y_pos_init, :]

        # store tempory points for a interesting area of the horizontal search
        points_found = []

        # list of lanes, which list is composed of a list with points for the lane
        lanes_list = []

        for index, pixel in enumerate(hor_line):

            if pixel == 255:
                points_found.append(index)

            if pixel == 0  and len(points_found) > 0:
                
                # after to save of all points of an area of instest, get the median 
                new_lane_center_point = np.array((int(np.median(points_found)), y_pos_init))
                points_found = []

                lanes_list.append(list([new_lane_center_point]))

        # from the points found in the first search, find all points of a lane 
        # using sliding window with a box of fix size
        for lane in lanes_list:

            y_pos_cur = y_pos_init + 1     

            x_lane_start = lane[-1][0]

            x_pos_cur = x_lane_start

            # go up
            while True:

                hor_line = image[y_pos_cur, 
                                x_pos_cur-4:x_pos_cur+4]

                hor_line_positions = [i for i in range(x_pos_cur-4,x_pos_cur+4)]


                for index, pixel in enumerate(hor_line):

                    if pixel == 255:
                        points_found.append(index)

                if len(points_found) > 0:

                    lane_center = hor_line_positions[int(np.median(points_found))]
                
                    # after to save of all points of an area of instest, get the median 
                    new_lane_center_point = np.array((lane_center, y_pos_cur))
                    points_found = []

                    lane.append(new_lane_center_point)

                    x_pos_cur = int(lane_center)
                    #print(lane_center)

                    #image = cv2.line(image, (x_pos_cur-5, y_pos_cur), (x_pos_cur+5, y_pos_cur), 255, 1)
                    #cv2.imshow("i", image)
                    #cv2.waitKey(1)

                # move window to the next line
                y_pos_cur = y_pos_cur - 1

                if y_pos_cur == 0:
                    break

            y_pos_cur = y_pos_init - 1

            x_pos_cur = x_lane_start

            points_found = []

            # go down
            while True:

                hor_line = image[y_pos_cur, 
                                x_pos_cur-4:x_pos_cur+4]

                hor_line_positions = [i for i in range(x_pos_cur-4,x_pos_cur+4)]


                for index, pixel in enumerate(hor_line):

                    if pixel == 255:
                        points_found.append(index)

                if len(points_found) > 0:
                    lane_center = hor_line_positions[int(np.median(points_found))]
                
                    # after to save of all points of an area of instest, get the median 
                    new_lane_center_point = np.array((lane_center, y_pos_cur))
                    points_found = []

                    lane.append(new_lane_center_point)

                    x_pos_cur = int(lane_center)

                    #image = cv2.line(image, (x_pos_cur-5, y_pos_cur), (x_pos_cur+5, y_pos_cur), 255, 1)
                    #cv2.imshow("i", image)
                    #cv2.waitKey(1)

                # move window to the next line
                y_pos_cur = y_pos_cur + 1 

                if y_pos_cur == (img_height-1):
                    break

        # fit a line for which lane
        for lane in lanes_list:

            lane_np = np.array(lane)

            x = lane_np[:, 0]
            y = lane_np[:, 1]

            z = np.polyfit(y, x, 2)
            p = np.poly1d(z)

            for i in range(400):

                p_x = int(p(i))
                p_y = i

                cv2.circle(image, (p_x, p_y), 2, 255, -1)
                cv2.imshow("i", image)
                cv2.waitKey(1)




    def find_lane(self, image):

        start = time.time()

        #self.follow_lanes(image, 400, 400, 100, 0)
        self.follow_lanes_both_sides(image, 400, 400, 150)

        p = time.time() - start

        fps = 1/p

        print(fps)

        return 

        init_line = [(0,400), (400,400)]

        y_pos = 399

        window_half_width = 5
        window_height = 40

        lanes_list = []

        start = time.time()

        while y_pos > 0:
            
            # percorre a imagem em n linhas horizontais
            hor_line = image[y_pos, :]


            lane_points = []

            for index, pixel in enumerate(hor_line):
                
                if pixel == 255:
                    lane_points.append(index)

                if pixel == 0 and len(lane_points) > 0:
                    new_point = np.array((int(np.median(lane_points)), y_pos))
                    lane_points = []

                    if len(lanes_list) == 0:
                        lanes_list.append(list([new_point]))

                    else:
                        # find if the new_point belongs to some lane found later
                        found_line_to_point = False

                        for lane in lanes_list:
                            last_point = lane[-1]

                            distancia = np.linalg.norm(last_point-new_point)

                            if distancia < 5:
                                lane.append(new_point)
                                found_line_to_point = True
                                break

                        if found_line_to_point == False:
                            lanes_list.append(list([new_point]))

            for lane in lanes_list:

                box_x_init = lane[-1][0]
                box_y_init = lane[-1][0]

                for box_y in range(100):

                    hor_box = image[box_y_init-box_y,
                                    box_x_init-window_half_width:box_x_init+window_half_width]

                    for b_index, b_pixel in enumerate(hor_box):

                        if b_pixel == 255:
                            lane_points.append(b_index)

                        if b_pixel == 0 and len(lane_points) > 0:
                            new_point = np.array((int(np.median(lane_points)), box_y_init-box_y))
                            lane_points = []

            print(lanes_list)

            y_pos -= 100
            #print(y_pos)

        t = time.time() - start
        fps = 1/t

        #print(fps)
        print("------------------------")
        
        
        #for p in lanes_list:
        #    cv2.circle(image, (p[0], p[1]), 4, 255, -1)
            

        cv2.imshow("image", image)
        cv2.waitKey(1)

    def histogram_view(self, image):
        
        (height, width) = image.shape

        new_image = image.copy()

        
        plt.ion()

        for l in range(height):
            line = image[l, :]

            new_image = cv2.line(new_image, (0,l), (width-1, l), (0,0,0), 10)
            cv2.waitKey(1)

            plt.plot(line)
            plt.draw()
            plt.pause(0.1)
            plt.clf()

        raw_input()
        
    def process(self, compressed_image):
        #self.filter_param.create_trackbar()
        #self.filter_param.update_trackbar_values()

        #image = self.bridge.compressed_imgmsg_to_cv2(compressed_image, "bgr8")
        image = compressed_image

        #if self.init == True:
        #    cv2.imwrite("/home/nesvera/image_test.jpg", image)

        bird_view = cv2.warpPerspective(image.copy(), self.homography_matrix, (400, 400))
        gray = cv2.cvtColor(bird_view, cv2.COLOR_BGR2GRAY)  

        #self.filter_param.update_trackbar_values()

        filtered = self.filter(gray)

        #self.histogram_view(filtered)
        self.find_lane(filtered)



        #cv2.imshow("image", image)
        #cv2.imshow("gray", gray)
        #cv2.imshow("bird view", filtered)
        #cv2.waitKey(1)

        if self.tune_param == 1:

            key = cv2.waitKey(1)&0xFF

            if key == ord('q'):
                cv2.destroyAllWindows()
                exit(1)

            elif key == ord('l'):
                homography.load_file()

            elif key == ord('s'):
                homography.save_file()
