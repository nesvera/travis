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

    SIGN_ID_NO_ENTRY     = 0
    SIGN_ID_DEAD_END     = 1
    SIGN_ID_RIGHT        = 2
    SIGN_ID_LEFT         = 3
    SIGN_ID_FORWARD      = 4
    SIGN_ID_STOP         = 5

    SIGN_DISTANCE_TO_STOP = 1.5

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
        self.motor_pid = PID(1, 0.1, 0, 50)

        self.lane_type = 0
        self.lane_offset = 0
        self.lane_curvature = 0

        self.cmd_steering = 0
        self.cmd_motor = 0

        self.ackermann_cmd = AckermannDrive()

        self.lane_offset_filter = deque(maxlen=5)

        self.signs_detected = []

    def routine(self):

        rate = rospy.Rate(10)
        
        # state machine
        while True:

            # Check buttons
            if self.btn_start:
                self.auto_enable = True

            elif self.btn_reset:
                self.auto_enable = False
                self.current_state = self.FSM_1_WAY

            elif self.btn_a:
                self.auto_race = True
                self.auto_urban = False

            elif self.btn_x:
                self.auto_race = False
                self.auto_urban = True

            elif self.btn_b:
                self.auto_race = False
                self.auto_urban = False

            if self.auto_enable:

                # steering
                self.lane_offset = sum(self.lane_offset_filter)/10.0
                self.cmd_steering = -self.steering_pid.update(0, self.lane_offset)
                self.cmd_steering = self.cmd_steering/100.0

                if self.cmd_steering >= 0:
                    self.ackermann_cmd.steering_angle = min(self.cmd_steering, 1)

                else:
                    self.ackermann_cmd.steering_angle = max(self.cmd_steering, -1)

                self.ackermann_cmd.speed = 0.4
                control_pub.publish(self.ackermann_cmd)

            # auto disable
            else:
                self.stop()

            rate.sleep()

    def stop(self):

        # stop
        self.ackermann_cmd.speed = -1
        self.ackermann_cmd.steering_angle = 0

        control_pub.publish(self.ackermann_cmd)

    def lane_detection_update(self, data):
        self.lane_type = data.lane_type
        self.lane_curvature = data.lane_curvature

        self.lane_offset_filter.append(data.lane_offset)

    def object_detection_update(self, data):
        self.object_status = data

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

def lane_detection_callback(data):
    control.lane_detection_update(data)

def object_detection_callback(data):
    control.object_detection_update(data)

def joystick_callback(data):
    control.joystick_update(data)

def exit_handler(signal, frame):
	exit(0)

if __name__ == '__main__':

    signal.signal(signal.SIGINT, exit_handler)

    # Initialize the node
    rospy.init_node('race')
    rospy.loginfo("Start race node")

    global control
    control = Control()

    # Publishers
    global control_pub
    control_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)

    # Subscribers
    rospy.Subscriber('/travis/lane_info', LaneInfo, lane_detection_callback, queue_size=1)
    rospy.Subscriber('/joy', Joy, joystick_callback)  

    control.routine()