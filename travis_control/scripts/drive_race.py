import rospy

from collections import deque

from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDrive

from travis_msg.msg import LaneInfo, SignInfo, SignsDetected

from pid import PID

class Control:

    FSM_INIT    = 0
    FSM_DRIVE   = 1
    FSM_RACE    = 2
    FSM_CROSS   = 3
    FSM_ZEBRA   = 4
    FSM_OBJECT  = 5

    def __init__(self):

        self.cross_status       = None
        self.lane_status        = deque(maxlen=30)
        self.object_status      = deque(maxlen=30)
        self.sign_status        = []
        self.zebra_status       = deque(maxlen=30)
        self.joystick_status    = None

        self.current_state = 0  

    def routine(self):

        rate = rospy.Rate(30)
        
        # state machine
        while True:

            if self.current_state == FSM_INIT:
                pass
            
            elif self.current_state == FSM_DRIVE:
                pass

            elif self.current_state == FSM_RACE:
                pass
            
            elif self.current_state == FSM_CROSS:
                pass

            elif self.current_state == FSM_ZEBRA:
                pass

            elif self.current_state == FSM_OBJECT:
                pass

            rate.sleep()

    def cross_detection_update(self, data):
        self.cross_status = data

    def lane_detection_update(self, data):
        self.lane_status = data

    def object_detection_update(self, data):
        self.object_status = data

    def sign_detection_update(self, data):
        self.sign_status = data

    def zebra_detection_update(self, data):
        self.zebra_status = data

    def joystick_update(self, data):
        self.joystick_status = data

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

if __name__ == '__main__':

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
    rospy.Subscriber('/lane_status', LaneInfo, lane_detection_callback, queue_size=1)
    rospy.Subscriber('/sign_status', SignsDetected, sign_detection_callback, queue_size=1)
    #rospy.Subscriber('/zebra_status', Bool, zebra_detection_callback, queue_size=1)
    
    control.routine()