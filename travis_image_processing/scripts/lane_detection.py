#!/usr/bin/env python

import rospy
import sys

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from travis_msg.msg import LaneInfo

import numpy as np
import cv2

from lane_detector import LaneDetector

def image_callback(data):
    lane_status = lane_detector.process(data)

    lane_status_pub.publish(lane_status)


if __name__ == "__main__":

    rospy.init_node('lane_detection')
    rospy.loginfo("Starting lane_detection.py")

    if len(sys.argv) < 2:
        rospy.loginfo("Error in lane_detection")
        rospy.loginfo("args 'homography_file' 'filter_file' ")
        exit(1)

    homography_file = sys.argv[1]
    #filter_file = "/home/nesvera/catkin_ws/src/travis/travis_image_processing/src/lane_detector/data/default.travis"
    filter_file = "/home/taura/catkin_ws/src/travis/travis_image_processing/src/lane_detector/data/lane.travis"
    debug = 1   

    global lane_detector
    lane_detector = LaneDetector(homography_file, filter_file, debug)

    bridge = CvBridge()

    # Publisher
    lane_status_pub = rospy.Publisher("/travis/lane_info", LaneInfo, queue_size=1)

    # Subscriber
    rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, image_callback, queue_size=1)

    if debug == 1:
        lane_detector.debug()

    else:
        rospy.spin()

