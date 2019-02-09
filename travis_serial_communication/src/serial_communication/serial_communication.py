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
        self.serialComm.timeout = 0.1
        
        self.stop = True

    def open(self):

        try:
            self.serialComm.open()

        except SerialException:
            print("Failed to open serial communication!")
            return False

        if self.serialComm.is_open == False:
            print("Serial port is closed!")
            return False

        return True

    def read(self):

        #self.serialComm.reset_input_buffer()

        while True:
            start = time.time()

            size = 0

            while size < 3:
                #receivedMsg = self.serialComm.readline()
                receivedMsg = self.serialComm.read_until('\n')         
                size = len(receivedMsg)
            

            p = time.time() -start

            print(p)



    def write(self, data):
        
        speed = float(data.speed)
        steering = float(data.steering_angle)

        msg = "*&" + str(throtle) + ";" + str(steering) + ";" + str(self.stop) + ";*"

        self.serialComm.write(msg)

    def write_message(self, message):

        self.serialComm.write(message)

