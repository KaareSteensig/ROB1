#!/usr/bin/env python
# license removed for brevity
import rospy
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Empty, Float32MultiArray

class is_at_position:
    def __init__(self):
        # set acceptabel diff before continueing
        self.acceptable = 0.3
        # prepare empty variables
        self.joint1error = None
        self.joint2error = None
        self.joint3error = None
        # prepare publisher
        self.pub = rospy.Publisher('/pathpointreached', Empty, queue_size = 1)
        print('Ready')

    def joint1callback(self, msg):
        print('joint1')
        self.joint1error = msg.error

    def joint2callback(self, msg):
        print('joint2')
        self.joint2error = msg.error

    def joint3callback(self, msg):
        print('joint3')
        self.joint3error = msg.error

    def request(self, msg):
        total = False
        print('request')
        while not total:
            joint1within = ((self.joint1error < self.acceptable) and (self.joint1error > -self.acceptable))
            joint2within = ((self.joint2error < self.acceptable) and (self.joint2error > -self.acceptable))
            joint3within = ((self.joint3error < self.acceptable) and (self.joint3error > -self.acceptable))
            #print(joint1within)
            #print(joint2within)
            #print(joint3within)
            total = joint1within and joint2within and joint3within
        print('continuing')
        self.pub.publish(Empty())

def start(nodeclass):
    # init node
    rospy.init_node('is_at_position', anonymous=True)
    # make subscribers
    sub = rospy.Subscriber('/position', Float32MultiArray, nodeclass.request)
    subJoint1 = rospy.Subscriber('/joint1/state', JointState, nodeclass.joint1callback)
    subJoint2 = rospy.Subscriber('/joint2/state', JointState, nodeclass.joint2callback)
    subJoint3 = rospy.Subscriber('/joint3/state', JointState, nodeclass.joint3callback)

if __name__ == '__main__':
    is_ready = is_at_position()
    start(is_ready)
    # spin node
    rospy.spin()
