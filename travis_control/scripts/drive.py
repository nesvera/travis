import rospy

from collections import deque
import time
import signal

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

from travis_msg.msg import LaneInfo, SignInfo, SignsDetected

from pid import PID

class Control:

    FSM_INIT    = 0
    FSM_1_WAY   = 1
    FSM_2_WAY   = 2
    FSM_CROSS   = 3
    FSM_ZEBRA   = 4
    FSM_OBJECT  = 5
    FSM_TURN    = 6
    FSM_OFF_ROAD = 7

    STOP        = -1
    DRIVE       = 0
    RACE        = 1

    ROAD_NOT_FOUND = 0
    ROAD_UNKOWN = 1
    ROAD_1_WAY = 2
    ROAD_2_WAY = 3

    def __init__(self):

        self.cross_status       = deque(maxlen=30)
        self.lane_type          = deque(maxlen=10)
        self.object_status      = deque(maxlen=30)
        self.sign_status        = []
        self.zebra_status       = deque(maxlen=30)
        self.joystick_status    = None
        self.lane_status        = None

        self.current_style = self.STOP
        self.current_state = self.FSM_INIT

        self.near_sign          = 0

        self.btn_start = False
        self.btn_reset = False
        self.btn_x = False
        self.btn_a = False
        self.btn_b = False

        self.auto_enable = False
        self.auto_race = False
        self.auto_urban = False

        self.steering_pid = PID(3, 0.1, 0, 50)
        self.motor_pid = PID(3, 0.1, 0, 50)

        self.lane_type = 0
        self.lane_offset = 0
        self.lane_curvature = 0

        self.cmd_steering = 0
        self.cmd_motor = 0

        self.ackermann_cmd = AckermannDrive()

        self.lane_offset_filter = deque(maxlen=5)

    def routine(self):

        rate = rospy.Rate(15)
        
        # state machine
        while True:

            #print(self.lane_offset, self.lane_curvature)

            # steering
            self.lane_offset = sum(self.lane_offset_filter)/10.0
            self.cmd_steering = -self.steering_pid.update(0, self.lane_offset)
            self.cmd_steering = self.cmd_steering/100.0

            if self.cmd_steering >= 0:
                self.ackermann_cmd.steering_angle = min(self.cmd_steering, 1)

            else:
                self.ackermann_cmd.steering_angle = max(self.cmd_steering, -1)

            self.ackermann_cmd.speed = self.cmd_motor

            control_pub.publish(self.ackermann_cmd)


            if self.current_state == self.FSM_INIT:

                time.sleep(3)
                print("Go")
                self.current_state = self.FSM_1_WAY   
            
            elif self.current_state == self.FSM_1_WAY:

                self.drive_1_way()

            elif self.current_state == self.FSM_2_WAY:

                self.drive_2_way()

            elif self.current_state == self.FSM_OFF_ROAD:
                pass
            
            elif self.current_state == self.FSM_CROSS:
                
                self.cross_handler()

                self.near_sign = 0


            elif self.current_state == self.FSM_ZEBRA:
                
                self.zebra_handler()


            elif self.current_state == self.FSM_OBJECT:

                self.object_handler()

            elif self.current_state == self.FSM_TURN:

                # direita
                self.turn(1)

                #esquerda
                self.turn(-1)

            # Check for zebra
            if len(self.zebra_status) > 5:
                self.current_state == self.FSM_ZEBRA

            # Check for crossing 
            if len(self.cross_status) > 5:
                self.current_state == self.FSM_CROSS

            # Check for traffic signs
            if self.near_sign != 0 and self.near_sign < 2:
                self.current_state == self.FSM_CROSS

            # Check for the type of road
            if self.lane_type == self.ROAD_NOT_FOUND:
                self.current_state = self.FSM_OFF_ROAD

            elif self.lane_type == self.ROAD_UNKOWN:
                self.current_state = self.FSM_2_WAY

            elif self.lane_type == self.ROAD_1_WAY:
                self.current_state = self.FSM_1_WAY

            elif self.lane_type == self.ROAD_2_WAY:
                self.current_state = self.FSM_2_WAY

            else:
                print("alguma coisa")

                pass

            # Check buttons
            if self.btn_start:
                self.auto_enable = True

            elif self.btn_reset:
                self.auto_enable = False

            elif self.btn_a:
                self.auto_race = True
                self.auto_urban = False

            elif self.btn_x:
                self.auto_race = False
                self.auto_urban = True

            elif self.btn_b:
                self.auto_race = False
                self.auto_urban = False


            rate.sleep()

    def drive_1_way(self):
        
        if self.auto_enable:
            self.steering_pid.update(0, 0)
            

    def drive_2_way(self):
        
        if self.auto_enable:
            self.steering_pid.update(0, 0)


    def cross_handler(self):
        
        self.stop()

        # wait 3 seconds
        self.current_state == self.FSM_TURN


    def zebra_handler(self):

        self.stop()

        # wait 3 seconds
        time.sleep(3)

        self.current_state == self.FSM_2_WAY

    def  object_handler(self):
        pass

    def stop(self):

        # stop
        cmd = AckermannDrive()
        cmd.speed = -1
        cmd.steering_angle = 0

    def turn(self, direction):

        cmd = AckermannDrive()
        cmd.speed = 0.8
        cmd.steering_angle = direction

    def cross_detection_update(self, data):
        self.cross_status = data

    def lane_detection_update(self, data):
        self.lane_type = data.lane_type
        self.lane_curvature = data.lane_curvature

        self.lane_offset_filter.append(data.lane_offset)

    def object_detection_update(self, data):
        self.object_status = data

    def sign_detection_update(self, data):
        self.sign_status = data

    def zebra_detection_update(self, data):
        self.zebra_status = data

    def joystick_update(self, data):

        self.btn_start = data.buttons[7]
        self.btn_reset = data.buttons[6]
        self.btn_x = data.buttons[2]
        self.btn_a = data.buttons[0]
        self.btn_b = data.buttons[1]

        # get data from the array
        left_trigger = data.axes[2]
        right_trigger = data.axes[5]

        # process throttle
        forward = (-right_trigger+1)/2.
        backward = (-left_trigger+1)/2.

        self.cmd_motor = 0
        if backward > 0.5:
            self.cmd_motor = -max(backward, -1)

        elif forward > 0.2:
            self.cmd_motor = min(forward, 1)

def cross_detection_callback(data):
    control.cross_detection_update(data)

def lane_detection_callback(data):
    control.lane_detection_update(data)

def object_detection_callback(data):
    control.object_detection_update(data)

def sign_detection_callback(data):
    control.sign_detection_update(data)

def zebra_detection_callback(data):
    control.zebra_detection_update(data)

def joystick_callback(data):
    control.joystick_update(data)

def exit_handler(signal, frame):
	exit(0)

if __name__ == '__main__':

    signal.signal(signal.SIGINT, exit_handler)

    # Initialize the node
    rospy.init_node('drive_race')
    rospy.loginfo("Start drive and race node")

    global control
    control = Control()

    # Publishers
    global control_pub
    control_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)

    # Subscribers
    #rospy.Subscriber('/cross_status', Bool, cross_detection_callback, queue_size=1)
    rospy.Subscriber('/travis/lane_info', LaneInfo, lane_detection_callback, queue_size=1)
    #rospy.Subscriber('/sign_status', SignsDetected, sign_detection_callback, queue_size=1)
    #rospy.Subscriber('/zebra_status', Bool, zebra_detection_callback, queue_size=1)
    rospy.Subscriber('/joy', Joy, joystick_callback)  

    control.routine()