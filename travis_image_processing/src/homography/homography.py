import numpy as np
import cv2
import pickle
import os
import sys

class Parameters:

    def __init__(self):

        # 4 points expressed in pixel
        self.screen_points = []

        # 4 points expressed in centimeters
        self.world_points = []

        # scale of cm/pixel
        # ex:   400 centimeters in axis x in real world
        #       1000 pixels in axis x of the image
        #       0.4 cm/pixel in bird-view
        self.scale_ratio = 1


        self.homography_matrix = None
        self.inverse_homography_matrix = None

        # transform parameters
        self.offset_x = 500
        self.offset_y = 500 

        self.afastamento = 500
        self.afastamento_scale = 500

        self.image_size = np.array((1280, 960))

    def set(self, dict):
        
        self.screen_points              = dict['screen_points']
        self.world_points               = dict['world_points']
        self.scale_ratio                = dict['scale_ratio']
        self.homography_matrix          = np.array(dict['homography_matrix'])
        self.inverse_homography_matrix  = np.array(dict['inverse_homography_matrix'])
        self.offset_x                   = dict['offset_x']
        self.offset_y                   = dict['offset_y']
        self.afastamento                = dict['afastamento']
        self.afastamento_scale          = dict['afastamento_scale']
        self.image_size                 = np.array(dict['image_size'])

class Homography:

    def __init__(self, file_path):
        self.parameters = Parameters()

        self.file_path = file_path

        if os.path.exists(self.file_path):
            self.load_file()


    def add_screen_point(self, x, y):
        
        if len(self.parameters.screen_points) < 4:
            self.parameters.screen_points.append((x,y))

        if len(self.parameters.screen_points) == 4:
            self.calculate_perspective_transform()

    def delete_last_screen_point(self):
        
        if len(self.parameters.screen_points) > 0:
            self.parameters.screen_points.pop()
            self.parameters.homography_matrix = None

    def reset_screen_points(self):
        
        del self.parameters.screen_points[:]
        self.parameters.homography_matrix = None

    def get_screen_points(self):
        
        return self.parameters.screen_points

    def set_world_points(self, points):

        self.parameters.world_points = points

    def set_parameters(self, offset_x, offset_y, ratio):

        self.parameters.offset_x = offset_x
        self.parameters.offset_y = offset_y
        self.parameters.afastamento = ratio
        self.parameters.scale_ratio = float(ratio)/self.parameters.afastamento_scale

        # if the matrix is already calculated, to it again
        if self.parameters.homography_matrix is not None:
            self.calculate_perspective_transform()

    def load_file(self):
        
        file_obj = open(self.file_path, 'r')
        
        try:
            parameters_dict = pickle.load(file_obj)
        except:
            print("Failed to open pickle file!")
            return

        self.parameters.set(parameters_dict)
        file_obj.close()

    def save_file(self):

        file_obj = open(self.file_path, 'w')
        class_to_dict = self.parameters.__dict__
        pickle.dump(class_to_dict, file_obj, pickle.HIGHEST_PROTOCOL)
        file_obj.close()

    def get_parameters_dict(self):

        return self.parameters.__dict__


    def calculate_perspective_transform(self):
        
        screen_points_np = np.asarray(self.parameters.screen_points, dtype="float32")
        world_points_np = np.asarray(self.parameters.world_points, dtype="float32")

        world_points_np *= self.parameters.scale_ratio

        offset_x = self.parameters.offset_x
        offset_y = self.parameters.offset_y

        world_points_np += [offset_x, offset_y]

        # 
        self.parameters.homography_matrix = cv2.getPerspectiveTransform(
            screen_points_np,
            world_points_np
        )

    def get_homography_matrix(self):
        
        return self.parameters.homography_matrix