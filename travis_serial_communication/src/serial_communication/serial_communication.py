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
        self.serialComm.port = '/dev/ttyUSB0'
        self.serialComm.baudrate = 115200
        self.serialComm.timeout = 4.0
        
        self.stop = 0

    def open(self):

        try:
            self.serialComm.open()

        except SerialException:
            print("Failed to open serial communication!")
            return False

        if self.serialComm.is_open == False:
            print("Serial port is closed!")
            return False

        self.serialComm.reset_input_buffer()

        return True

    def read(self):

        #self.serialComm.reset_input_buffer()

        receivedMsg = self.serialComm.read_until('*')
        #print(receivedMsg)

        return 

        while True:
            start = time.time()

            size = 0

            receivedMsg = self.serialComm.read_until('*')

            p = time.time() -start

            print(receivedMsg)

    def write_ackermann(self, data):
        
        speed = round(data.speed, 2)
        steering = round(data.steering_angle, 2)

        msg = "&" + str(speed) + ";" + str(steering) + ";" + str(self.stop) + ";*"

        self.write_message(msg)

    def write_message(self, message):

        self.serialComm.write(message)

