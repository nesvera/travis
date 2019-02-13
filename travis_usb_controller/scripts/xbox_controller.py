#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

import sys
import time

fixed_speed_value = 0
max_speed = 1
max_steering = 1

class Controller:

    def __init__(self, max_speed, max_steering):

        self.max_speed = max_speed
        self.max_steering = max_steering

        self.speed = 0
        self.steering  = 0

        self.axes = None
        self.buttons = None

        self.new_data = False

    def update(self, data):

        self.axes = data.axes
        self.buttons = data.buttons

        # get data from the array
        left_trigger = data.axes[2]
        right_trigger = data.axes[5]
        left_x_stick = -data.axes[0]

        # process throttle
        forward = (-right_trigger+1)/2.
        backward = (-left_trigger+1)/2.

        speed = 0
        if backward > 0.5:
            speed = -max(backward, -1)

        elif forward > 0.2:
            speed = min(forward, 1)*max_speed

        # process steering
        steering = 0
        if left_x_stick > 0.4:
            steering = min(left_x_stick, 1)*max_steering

        elif left_x_stick < -0.4:
            steering = max(left_x_stick, -1)*max_steering

        ackermann_cmd.speed = speed
        ackermann_cmd.steering_angle = steering

        #print(ackermann_cmd.speed, ackermann_cmd.steering_angle)

        self.new_data = True

    def routine(self):

        rate = rospy.Rate(30)

        timer_start = time.time()

        while not rospy.is_shutdown():
            
            if self.new_data or (time.time() - timer_start) < 2:
                ackermann_pub.publish(ackermann_cmd)
                #print(ackermann_cmd)

                if self.new_data:
                    timer_start = time.time()

                self.new_data = False

            rate.sleep()

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

