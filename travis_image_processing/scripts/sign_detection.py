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

    global sign_detector
    sign_detector = SignDetector(8, True)

    image = None

    img_sub = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, image_callback)

    #rospy.spin()

    bridge = CvBridge()
    rate = rospy.Rate(30)

    #image = cv2.imread("/home/nesvera/image_test.jpg", cv2.IMREAD_COLOR)
    cap = cv2.VideoCapture(0)

    while True:
        ret, image = cap.read()
        sign_detector.process(image.copy())

        rate.sleep()


