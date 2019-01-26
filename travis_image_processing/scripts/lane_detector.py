#!/usr/bin/env python

import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2

def image_callback(data):
    global image

    # convert received image to BGR
    image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")

if __name__ == "__main__":

    rospy.init_node('lane_detector')
    rospy.loginfo("Starting land_detector.py")

    image = None

    img_sub = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage, image_callback)
    bridge = CvBridge()    
    

    while True:

        if image is not None:
            cv2.imshow('image', image)
        
            if cv2.waitKey(1)&0xFF == ord('q'):
                exit(0)