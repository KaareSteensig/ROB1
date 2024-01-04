#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray, Float64



def callback(data,args):

    pub1 = args[0]
    pub2 = args[1]
    pub3 = args[2]

    rospy.loginfo(rospy.get_caller_id() + "I heard X=%f and Y=%f and Z=%f", data.data[0],data.data[1],data.data[2])

    L1 = 173
    L2 = 250

    X = data.data[0]
    Y = data.data[1]
    Z = data.data[2]

    theta0 = math.atan2(Y,X)

    #rospy.loginfo(rospy.get_caller_id() + " theta0 - %f", theta0/(2*math.pi)*360)

    P = math.sqrt(X*X+Y*Y)
    
    #rospy.loginfo(rospy.get_caller_id() + " P - %f", P)

    L = math.sqrt(P*P+Z*Z)

    #rospy.loginfo(rospy.get_caller_id() + " L - %f", L)

    #test = (L1**2+L2**2-L**2)/(2*L1*L2)

    theta2 = -(math.acos((L1*L1+L2*L2-L*L)/(2*L1*L2)))+math.pi

    rospy.loginfo(rospy.get_caller_id() + " theta2 - %f", theta2/(2*math.pi)*360)

    alpha = math.acos((L*L+L1*L1-L2*L2)/(2*L*L1))

    rospy.loginfo(rospy.get_caller_id() + " alpha - %f", alpha/(2*math.pi)*360)

    beta = math.atan2(Z,P)

    rospy.loginfo(rospy.get_caller_id() + " beta - %f", beta/(2*math.pi)*360)

    theta1 = -(alpha + beta)+(math.pi/2)

    rospy.loginfo(rospy.get_caller_id() + " theta1 - %f", theta1/(2*math.pi)*360)

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
