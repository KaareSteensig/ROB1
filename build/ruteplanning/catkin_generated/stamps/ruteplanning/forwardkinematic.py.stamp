#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float32MultiArray, Bool
from dynamixel_msgs.msg import JointState
from math import sin, cos, atan2, sqrt, pi

class ForwardKinematics:
    def __init__(self):
        rospy.init_node('forward_kinematics')

        # Subscriber for getting calculation of end point
        rospy.Subscriber('/endPointReached', Bool, self.calculate_fk)

        # Subscriber for joint states
        rospy.Subscriber('/joint1/state', JointState, self.joint1_callback)
        rospy.Subscriber('/joint2/state', JointState, self.joint2_callback)
        rospy.Subscriber('/joint3/state', JointState, self.joint3_callback)

        self.joint_positions = [0.0, 0.0, 0.0]  # Initialize joint positions

        # Publisher for current position
        self.current_position_publisher = rospy.Publisher('/currentPosition', Float32MultiArray, queue_size=10)

    def joint1_callback(self, msg):
        self.joint_positions[0] = msg.current_pos

    def joint2_callback(self, msg):
        self.joint_positions[1] = msg.current_pos

    def joint3_callback(self, msg):
        self.joint_positions[2] = msg.current_pos

    def calculate_fk(self, msg):
        L1 = 0  # Length from joint 1 to joint 2
        L2 = 173.0  # Length from joint 2 to joint 3
        L3 = 215.0  # Length from joint 3 to end-effector (assuming a fixed end-effector length)

        theta1, theta2, theta3 = self.joint_positions
        theta2 = 3.14/2 - theta2
        theta3 = 3.14 - theta3
        rospy.loginfo("Joint Position: theta1=%.2f, theta2=%.2f, theta3=%.2f", theta1, theta2, theta3)

        # Calculate forward kinematics based on the known link lengths
        # Using https://medium.com/@ringlayer/forward-kinematics-calculation-for-robotic-arm-6393934f847
        # Z-axis
        d3 = sin(theta2) * L2
        theta4 = 3.14 - ((3.14/2 - theta2) + 3.14/2)
        theta5 = 3.14 - (theta3 + theta4)
        d6 = sin(theta5) * L3
        z = L1 + d3 - d6
        # X-axis
        #x = L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3)
        d4 = cos(theta2) * L2
        d5 = cos(theta5) * L3
        d1 = d4 + d5
        x = cos(theta1) * d1
        # Y-axis
        #y = L1 * sin(theta1) + L2 * sin(theta1 + theta2) + L3 * sin(theta1 + theta2 + theta3)
        y = sin(theta1) * d1

        rospy.loginfo("End Effector Position: x=%.2f, y=%.2f, z=%.2f", x, y, z)

        # Publish the current position
        current_position_msg = Float32MultiArray(data=[x, y, z])
        self.current_position_publisher.publish(current_position_msg)

def main():
    forward_kinematics = ForwardKinematics()
    rospy.spin()

if __name__ == '__main__':
    main()

