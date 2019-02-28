#!/usr/bin/env python

import rospy
import sys

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2

from traffic_sign_detector import SignDetector

from travis_msg.msg import SignsDetected

def image_callback(data):

    # convert received image to BGR
    image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")

    sings_detected = sign_detector.process(image)

    sign_status_pub.publish(sings_detected)

if __name__ == "__main__":
    
    rospy.init_node('sign_detection')
    rospy.loginfo("Starting sign_detection.py")

    debug = 1

    global sign_detector
    sign_detector = SignDetector(0.08)

    bridge = CvBridge()

    # Publisher
    sign_status_pub = rospy.Publisher("/travis/sign_detected", SignsDetected, queue_size=1)

    # Subscriber
    rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, image_callback)

    if debug == 1:
        sign_detector.debug()

    else:
        rospy.spin()


