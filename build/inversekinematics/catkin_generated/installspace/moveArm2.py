#!/usr/bin/env python2
import rospy
import math
from std_msgs.msg import Float32MultiArray, Float64



def callback(data,args):

    pub1 = args[0]
    pub2 = args[1]
    pub3 = args[2]

    rospy.loginfo(rospy.get_caller_id() + "I heard X=%f and Y=%f and Z=%f", data.data[0],data.data[1],data.data[2])

    #L1 = 173
    #L2 = 215

    X = data.data[0]
    Y = data.data[1]
    Z = data.data[2]

    # Calculate q1
    theta0 = math.atan2(Y, X)

    # Parameters
    a1 = 17  # Replace with the value of 'a1'
    d1 = 12 # Replace with the value of 'd1'
    a2 = 21 # Replace with the value of 'a2'
    d4 = 1 # Replace with the value of 'd4'

    # Calculate q2 and q3
    r2 = (X - a1 * math.cos(theta0))**2 + (Y - a1 * math.sin(theta0))**2
    s = Z - d1
    D = (r2 + s**2 - a2**2 - d4**2) / (2 * a2 * d4)

    theta2 = math.atan2(-math.sqrt(1 - D**2), D)
    theta1 = math.atan2(s, math.sqrt(r2)) - math.atan2(d4 * math.sin(theta2), a2 + d4 * math.cos(theta2))


    pub1.publish(theta0)
    pub2.publish(theta1)
    pub3.publish(theta2)


def subscriberAndPublisher():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    pubJoint1 = rospy.Publisher('/joint1/command',Float64)
    pubJoint2 = rospy.Publisher('/joint2/command',Float64)
    pubJoint3 = rospy.Publisher('/joint3/command',Float64)
    #rate = rospy.Rate(10)

    rospy.init_node('armMover', anonymous=True)

    rospy.Subscriber("position", Float32MultiArray, callback,(pubJoint1,pubJoint2,pubJoint3))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriberAndPublisher()
