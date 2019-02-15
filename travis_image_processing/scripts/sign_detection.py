#!/usr/bin/env python

import rospy
import sys

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2

from traffic_sign_detector import SignDetector

def image_callback(data):

    # convert received image to BGR
    #image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")

    sign_detector.process(data)

if __name__ == "__main__":
    
    rospy.init_node('sign_detection')
    rospy.loginfo("Starting sign_detection.py")

    if len(sys.argv) < 2:
        rospy.loginfo("Error in sign_detection")
        rospy.loginfo("Cant find json file!")
        exit(1)

    homography_file = sys.argv[1]
    filter_file = "/home/nesvera/catkin_ws/src/travis/travis_image_processing/src/lane_detector/data/default.travis"
    

    global sign_detector
    sign_detector = SignDetector(homography_file, filter_file, 1)

    image = None

    img_sub = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, image_callback)

    #rospy.spin()

    bridge = CvBridge()
    rate = rospy.Rate(10)

    image = cv2.imread("/home/nesvera/image_test.jpg", cv2.IMREAD_COLOR)

    while True:
        sign_detector.process(image.copy())

        #cv2.imshow("cu", image)
        #cv2.waitKey(1)

        rate.sleep()


