#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32MultiArray

class coordinatechange:
    def __init__(self):
        # prepare variables
        self.offsets = [75.0, 75.0]
        # prepare publisher
        self.pub = rospy.Publisher('/goalPosition', Float32MultiArray, queue_size = 1)
        # prepare subscriber
        self.sub = rospy.Subscriber('/gcode_position', Float32MultiArray, self.converter)
        self.sub = rospy.Subscriber('/offset', Float32MultiArray, self.offset)

    def converter(self, msg):
        corrected_position = [msg.data[0]+self.offsets[0], msg.data[1]+self.offsets[1], msg.data[2]-145]
        self.pub.publish(Float32MultiArray(data = corrected_position))

    def offset(self, msg):
        off0 = msg.data[0]
        self.offsets[0] = off0
        off1 = msg.data[1]
        self.offsets[1] = off1
        print("offset updated")

def start():
    # init node
    rospy.init_node('coordinatechange', anonymous=True)
    changer = coordinatechange()

if __name__ == '__main__':
    start()
    # spin node
    rospy.spin()
