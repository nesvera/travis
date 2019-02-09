#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

import sys
import sys

fixed_speed_value = 0
max_speed = 0
max_steering = 1

class Controller:

    def __init__(self, max_speed, max_steering):

        self.max_speed = max_speed
        self.max_steering = max_steering

        self.speed = 0
        self.steering  = 0

        self.axes = None
        self.buttons = None

    def update(self, data):

        self.axes = data.axes
        self.buttons = data.buttons

        left_trigger = data.axes[2]
        right_trigger = data.axes[5]
        left_x_stick = data.axes[0]

        forward = (-right_trigger+1)/2.
        backward = (-left_trigger+1)/2.

        speed = forward*max_speed
        steering = left_x_stick*max_steering

        print(speed, steering)

        ackermann_cmd.speed = float(speed)
        ackermann_cmd.steering_angle = float(steering)

        new_data = True

    def routine(self):

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            
            ackermann_pub.publish(ackermann_cmd)

def joy_callback(data):
    
    controller.update(data)         

if __name__ == "__main__":

    rospy.init_node('xbox_controller')
    rospy.loginfo("Starting xbox_controller.py")

    max_speed = int(sys.argv[1])
    max_steering = int(sys.argv[2])

    controller = Controller(max_speed, max_steering)

    # Subscribe to the topic that contains the controller keys
    rospy.Subscriber('/joy', Joy, joy_callback)      

    global ackermann_pub, ackermann_cmd
    ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
    ackermann_cmd = AckermannDrive()

    # infinite loop sending commands from controller to the car
    controller.routine()

