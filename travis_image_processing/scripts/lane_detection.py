#!/usr/bin/env python

import rospy
import sys

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2

from lane_detector import LaneDetector

def image_callback(data):

    # convert received image to BGR
    #image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")

    lane_detector.process(data)

if __name__ == "__main__":

    rospy.init_node('lane_detection')
    rospy.loginfo("Starting land_detection.py")

    if len(sys.argv) < 2:
        rospy.loginfo("Error in lane_detection")
        rospy.loginfo("Cant find json file!")
        exit(1)

    homography_file = sys.argv[1]

    global lane_detector
    lane_detector = LaneDetector(homography_file)

    image = None

    img_sub = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, image_callback)

    rospy.spin()