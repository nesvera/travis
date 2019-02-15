#!/usr/bin/env python

import rospy
import sys

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2

from zebra_detector import ZebraDetector

def image_callback(data):

    # convert received image to BGR
    #image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")

    zebra_detector.process(data)

if __name__ == "__main__":
    
    rospy.init_node('zebra_detection')
    rospy.loginfo("Starting zebra_detection.py")

    if len(sys.argv) < 2:
        rospy.loginfo("Error in zebra_detection")
        rospy.loginfo("Cant find json file!")
        exit(1)

    homography_file = sys.argv[1]
    filter_file = "/home/nesvera/catkin_ws/src/travis/travis_image_processing/src/zebra_detector/data/default.travis"
    

    global zebra_detector
    zebra_detector = ZebraDetector(homography_file, filter_file, 1)

    image = None

    img_sub = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, image_callback)

    #rospy.spin()

    bridge = CvBridge()
    rate = rospy.Rate(10)

    image = cv2.imread("/home/nesvera/image_test.jpg", cv2.IMREAD_COLOR)

    while True:
        zebra_detector.process(image.copy())

        rate.sleep()


