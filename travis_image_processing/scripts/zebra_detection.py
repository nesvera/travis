#!/usr/bin/env python

import rospy
import sys

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

import numpy as np
import cv2

from zebra_detector import ZebraDetector

def image_callback(data):
    zebra_status = zebra_detector.process(data)
    zebra_status_pub.publish(zebra_status)

if __name__ == "__main__":
    
    rospy.init_node('zebra_detection')
    rospy.loginfo("Starting zebra_detection.py")

    if len(sys.argv) < 2:
        rospy.loginfo("Error in zebra_detection")
        rospy.loginfo("args 'homography_file' 'filter_file' 'debug'")
        exit(1)

    homography_file = sys.argv[1]
    #filter_file = "/home/nesvera/catkin_ws/src/travis/travis_image_processing/src/zebra_detector/data/default.travis"
    filter_file = "/home/taura/catkin_ws/src/travis/travis_image_processing/src/zebra_detector/data/zebra.travis"
    debug = 1

    global zebra_detector
    zebra_detector = ZebraDetector(homography_file, filter_file, debug)

    bridge = CvBridge()

    # Publisher
    zebra_status_pub = rospy.Publisher("/travis/zebra_status", Bool, queue_size=1)

    # Subscriber
    rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, image_callback)
      