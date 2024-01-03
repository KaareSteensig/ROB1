#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Float64
import curses

def main(stdscr):
    # Initialize the ROS node
    rospy.init_node('keyboard_input_node', anonymous=True)

    # Create a publisher for the '/position' topic
    pub_position = rospy.Publisher('/position', Float32MultiArray, queue_size=10)
    pub_gripper = rospy.Publisher('/gripper/command', Float64, queue_size=1)
    pub_joint4 = rospy.Publisher('/joint4/command', Float64, queue_size=1)

    # Initialize the Float32MultiArray message with default values
    position = Float32MultiArray()
    position.data = [150.0, 150.0, 150.0]  # Default X, Y, and Z

    moveDistance = 20.0

    # Initialize the rate with a 10 Hz maximum publishing rate
    max_publish_rate = rospy.Rate(5)

    gripper_command = 0.0
    joint4_command = 0.0

    last_gripper_command = gripper_command  # Store the last gripper command

    # Initialize curses
    stdscr.nodelay(1)
    stdscr.timeout(100)  # 100ms timeout for non-blocking getch
    curses.curs_set(0)  # Hide the cursor

    while not rospy.is_shutdown():
        key = stdscr.getch()

        x_changed, y_changed, z_changed = False, False, False

        if key == curses.KEY_UP:
            position.data[0] -= moveDistance  # Update X
            x_changed = True
        elif key == curses.KEY_DOWN:
            position.data[0] += moveDistance  # Update X
            x_changed = True
        elif key == curses.KEY_LEFT:
            position.data[1] -= moveDistance  # Update Y
            y_changed = True
        elif key == curses.KEY_RIGHT:
            position.data[1] += moveDistance  # Update Y
            y_changed = True
        elif key == ord(' '):
            position.data[2] += moveDistance  # Update Z
            z_changed = True
        elif key == ord('c'):
            position.data[2] -= moveDistance  # Update Z
            z_changed = True

        if x_changed or y_changed or z_changed:
            pub_position.publish(position)
            max_publish_rate.sleep()

        if key == ord('o') and gripper_command != 0.0:
            gripper_command = 0.0
        elif key == ord('p') and gripper_command != 1.0:
            gripper_command = 1.0

        if gripper_command != last_gripper_command:
            pub_gripper.publish(gripper_command)
            last_gripper_command = gripper_command

        if key == ord('m'):
            joint4_command = max(-2.0, joint4_command - 0.1)
        elif key == ord('n'):
            joint4_command = min(2.0, joint4_command + 0.1)

        pub_joint4.publish(joint4_command)

if __name__ == '__main__':
    try:
        stdscr = curses.initscr()
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass

