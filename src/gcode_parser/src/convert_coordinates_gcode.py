#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64, Empty, Float32MultiArray

class coordinatechange:
    def __init__(self):
        # prepare publisher
        self.pub = rospy.Publisher('/position', Float32MultiArray, queue_size = 1)
        # prepare subscriber
        self.sub = rospy.Subscriber('/gcode_position', Float32MultiArray, self.converter)

    def converter(self, msg):
        corrected_position = [msg.data[0]+60, msg.data[1]+60, msg.data[2]-50]
        self.pub.publish(Float32MultiArray(data = corrected_position))

def start():
    # init node
    rospy.init_node('coordinatechange', anonymous=True)
    changer = coordinatechange()

if __name__ == '__main__':
    start()
    # spin node
    rospy.spin()
