import rospy

from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy

class Control:

    def __init__(self):

        pass

    def routine(self):
        
        pass

    def lane_detection_update(self, data):

        pass

    def object_detection_update(self, data):

        pass

    def joystick_update(self, data):

        pass

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



    rospy.spin()