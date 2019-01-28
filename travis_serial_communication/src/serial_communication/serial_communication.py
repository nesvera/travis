import numpy as np

import rospy
from std_msgs.msg import Int32
from ackermann_msgs.msg import AckermannDrive

import serial
from serial import SerialException
import time

# Jetson to arduino variables
throtle = 0
steering = 0
jetsonStop = False

throtleMin = -100
throtleMax = 100
steeringMin = -100
steeringMax = 100

# Arduino to Jetson variables
aproxSpeed = 0                              # speed from hall effect sensor
hallCounter = 0                             # number of pulses from hall effect sensor
accelX = 0
accelY = 0
accelZ = 0
gyroRoll = 0
gyroPitch = 0
gyroYaw = 0
radioStop = False 

class SerialCommunication:

    def __init__(self):

        self.serialComm = serial.Serial()
        self.serialComm.port = '/dev/ttyACM0'
        self.serialComm.baudrate = 115200
        self.serialComm.timeout = 0.5

    def open(self):

        try:
            self.serialComm.open()

        except SerialException:
            print("Failed to open serial communication!")
            return False

        if serialComm.is_open == False:
            print("Serial port is closed!")
            return False

        return True

    def read(self):
        
        while True:

            receivedMsg = self.serialComm.readline()
            print(receivedMsg)
            pass

    def write(self, data):
        
        speed = float(data.speed)
        steering = float(data.steering_angle)

        print(speed, steering)

        msg = "*&" + str(int(throtle)) + ";" + str(int(steering)) + ";" + str(int(jetsonStop)) + ";*"

        self.serialComm.write(msg)
