#!/usr/bin/env python

import rospy
import sys

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2

from homography import Homography

def image_callback(data):
    global image, new_image

    # convert received image to BGR
    image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    new_image = True
    
def mouse_handler(event, x, y, flags, param):

    if event == cv2.EVENT_LBUTTONUP:
        homography.add_screen_point(x, y)

def trackbar_update(arg):
    offset_x = cv2.getTrackbarPos("Offset x", "Parametros") - trackbar_offset
    offset_y = cv2.getTrackbarPos("Offset y", "Parametros") - trackbar_offset
    afastamento = cv2.getTrackbarPos("Afastamento", "Parametros")
    homography.set_parameters(offset_x, offset_y, afastamento)

def create_trackbar():
    cv2.namedWindow("camera_image")
    cv2.setMouseCallback("camera_image", mouse_handler)

    global trackbar_size, trackbar_offset
    trackbar_size =  1000
    trackbar_offset = trackbar_size/2.
    cv2.namedWindow("Parametros")
    cv2.createTrackbar("Offset x",      "Parametros", 0, trackbar_size, trackbar_update)
    cv2.createTrackbar("Offset y",      "Parametros", 0, trackbar_size, trackbar_update)
    cv2.createTrackbar("Afastamento",   "Parametros", 0, trackbar_size, trackbar_update)

    parameters = homography.get_parameters_dict()

    offset_x = parameters['offset_x']
    offset_y = parameters['offset_y']
    afastamento = parameters['afastamento']

    cv2.setTrackbarPos("Offset x",      "Parametros", int(offset_x))
    cv2.setTrackbarPos("Offset y",      "Parametros", int(offset_y))
    cv2.setTrackbarPos("Afastamento",   "Parametros", int(afastamento))

if __name__ == "__main__":
    global homography

    rospy.init_node('find_homography')
    rospy.loginfo("Starting find_homography.py")

    if len(sys.argv) < 2:
        rospy.loginfo("Error in find_homography!")
        rospy.loginfo("Cant find json file!")
        exit(1)

    file_path = sys.argv[1]

    print(file_path)

    homography = Homography(file_path)

    # create trackbar and set values
    create_trackbar()

    # set world points
    world_points = [
        [000.0, 800.0],
        [400.0, 800.0],
        [400.0, 600.0],
        [000.0, 600.0]]

    homography.set_world_points(world_points)

    new_image = False

    img_sub = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, image_callback)
    bridge = CvBridge()

    np.set_printoptions(suppress=True)

    while True:

        if new_image:
            original = image.copy()
            cv2.circle(original, (640, 800), 8, (0,255,0), -1)

            # draw selected screen_points
            screen_points = homography.get_screen_points()
            for p in screen_points:
                cv2.circle(image, p, 5, (0,255,0), -1)

            # bird-view
            homography_matrix = homography.get_homography_matrix()

            if homography_matrix is not None:
                bird_view = cv2.warpPerspective(original, homography_matrix, (1280, 960))
                cv2.circle(bird_view, (640, 960), 5, (0,255,0), -1)

                cv2.imshow("warped", bird_view)

            cv2.imshow("camera_image", image)


        key = cv2.waitKey(1)&0xFF

        if key == ord('q'):
            cv2.destroyAllWindows()
            break

        elif key == ord('r'):
            homography.reset_screen_points()

        elif key == ord('l'):
            homography.load_file()

        elif key == ord('s'):
            homography.save_file()

        elif key == ord('d'):
            homography.delete_last_screen_point()