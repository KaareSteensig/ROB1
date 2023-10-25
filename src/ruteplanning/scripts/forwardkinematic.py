#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float32MultiArray
from dynamixel_msgs.msg import JointState
from math import sin, cos, atan2, sqrt, pi

class ForwardKinematics:
    def __init__(self):
        rospy.init_node('forward_kinematics')

        # Subscriber for joint states
        rospy.Subscriber('/joint1/state', JointState, self.joint1_callback)
        rospy.Subscriber('/joint2/state', JointState, self.joint2_callback)
        rospy.Subscriber('/joint3/state', JointState, self.joint3_callback)

        self.joint_positions = [0.0, 0.0, 0.0]  # Initialize joint positions

        # Publisher for current position
        self.current_position_publisher = rospy.Publisher('/currentPosition', Float32MultiArray, queue_size=10)

    def joint1_callback(self, data):
        self.joint_positions[0] = data.current_pos
        self.calculate_fk()

    def joint2_callback(self, data):
        self.joint_positions[1] = data.current_pos
        self.calculate_fk()

    def joint3_callback(self, data):
        self.joint_positions[2] = data.current_pos
        self.calculate_fk()

    def calculate_fk(self):
        L1 = 173.0  # Length from joint 1 to joint 2
        L2 = 215.0  # Length from joint 2 to joint 3
        L3 = 100.0  # Length from joint 3 to end-effector (assuming a fixed end-effector length)

        theta1, theta2, theta3 = self.joint_positions

        # Calculate forward kinematics based on the known link lengths
        x = L1 * cos(theta1) + L2 * cos(theta1 + theta2) + L3 * cos(theta1 + theta2 + theta3)
        y = L1 * sin(theta1) + L2 * sin(theta1 + theta2) + L3 * sin(theta1 + theta2 + theta3)
        z = 0.0  # Assuming a planar robot (z remains constant)

        rospy.loginfo("End Effector Position: x=%.2f, y=%.2f, z=%.2f", x, y, z)

        # Publish the current position
        current_position_msg = Float32MultiArray(data=[x, y, z])
        self.current_position_publisher.publish(current_position_msg)

def main():
    forward_kinematics = ForwardKinematics()
    rospy.spin()

if __name__ == '__main__':
    main()

