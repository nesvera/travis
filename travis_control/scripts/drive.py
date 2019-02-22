import rospy

from collections import deque

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

    STOP        = -1
    DRIVE       = 0
    RACE        = 1

    def __init__(self):

        self.cross_status       = deque(maxlen=30)
        self.lane_status        = None
        self.lane_type          = deque(maxlen=10)
        self.object_status      = deque(maxlen=30)
        self.sign_status        = []
        self.zebra_status       = deque(maxlen=30)
        self.joystick_status    = None

        self.current_style = self.STOP
        self.current_state = self.FSM_INIT

        self.near_sign          = 0

    def routine(self):

        rate = rospy.Rate(30)
        
        # state machine
        while True:

            if self.current_state == self.FSM_INIT:
                
                # esperando clicar botoes do controle para ativar carrinho

                pass
            
            elif self.current_state == self.FSM_1_WAY:

                self.drive_1_way()


            elif self.current_state == self.FSM_2_WAY:

                self.drive_2_way()

            
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

            rate.sleep()

    def drive_1_way(self):
        pass

    def drive_2_way(self):
        pass

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
    rospy.Subscriber('/cross_status', Bool, cross_detection_callback, queue_size=1)
    rospy.Subscriber('/lane_status', LaneInfo, lane_detection_callback, queue_size=1)
    rospy.Subscriber('/sign_status', SignsDetected, sign_detection_callback, queue_size=1)
    rospy.Subscriber('/zebra_status', Bool, zebra_detection_callback, queue_size=1)
    
    control.routine()