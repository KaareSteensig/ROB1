#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def joint1pubber():
    pub = rospy.Publisher('/joint1/command', Float64, queue_size=10)
    rospy.init_node('joint1publisher', anonymous=True)
    rate = rospy.Rate(0.5)
    move_radians = 0.5
    while not rospy.is_shutdown():
        move_radians = -move_radians
        rospy.loginfo(move_radians)
        pub.publish(move_radians)
        rate.sleep()

if __name__ == '__main__':
    try:
        joint1pubber()
    except rospy.ROSInterruptException:
        pass
