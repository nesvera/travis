#!/usr/bin/env python

'''
    This node implements the communication between the computer and
    the controller board.

    -> Open serial communication with the controller board
    -> Get ackermann data and send to the controller board
    -> Read controller board data from the car
'''

import rospy
from ackermann_msgs.msg import AckermannDrive

import time

from serial_communication import SerialCommunication

# Callback to send data to arduino
def drive_callback(data):
    serialComm.write_ackermann(data)

def routine():

    while True:

        serialComm.read()

if __name__ == '__main__':
    global serialComm

    # Initialize the node
    rospy.init_node('serial_communication')
    rospy.loginfo("Start serial communication!")

    serialComm = SerialCommunication()

    if serialComm.open() == False:
        exit(1)

    rospy.loginfo("Opened serial communication!")

    rospy.Subscriber('/ackermann_cmd', AckermannDrive, drive_callback)

    # infinite loop reading data that comes from the microcontroller
    routine()