#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

import sys
import argparse

fixed_speed = False
fixed_speed_value = 0
max_speed = 0
max_steering = 100

def joy_callback(data):
    global new_data, ackermann_cmd

    axes = data.axes
    buttons = data.buttons

    left_trigger = data.axes[2]
    right_trigger = data.axes[5]-0.134
    left_x_stick = data.axes[0]

    forward = (-right_trigger+1)/2.
    backward = (-left_trigger+1)/2.

    speed = forward*max_speed
    steering = left_x_stick*max_steering

    print(speed, steering)

    ackermann_cmd.speed = float(speed)
    ackermann_cmd.steering_angle = float(steering)

    new_data = True


def routine():
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        ackermann_pub.publish(ackermann_cmd)

        rate.sleep()
            

if __name__ == "__main__":

    rospy.init_node('xbox_controller')
    rospy.loginfo("Starting xbox_controller.py")

    parser = argparse.ArgumentParser()
    parser.add_argument('-m ', '--max_speed', action='store', dest='max_speed',
                        default=100, required=False,
                        help="Limit of speed.")
    
    arguments = parser.parse_args(rospy.myargv()[1:])

    max_speed = float(arguments.max_speed)    

    # Subscribe to the topic that contains the controller keys
    rospy.Subscriber('/joy', Joy, joy_callback)      

    global ackermann_pub, ackermann_cmd
    ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
    ackermann_cmd = AckermannDrive()

    # infinite loop sending commands from controller to the car
    routine()

