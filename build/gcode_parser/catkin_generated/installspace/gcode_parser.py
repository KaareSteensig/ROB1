#!/usr/bin/env python2
import rospy
from std_msgs.msg import String, Float32MultiArray
from custom_msgs.msg import GCodeInfo  # You may need to create a custom message

# Define variables to store the current state of coordinates, feed rate, and extruder status.
current_x, current_y, current_z, current_feed_rate, current_extrusion, extruder_on = 1, 1, 1, None, None, False
extruder_on = False  # Assume extruder is initially off

def update_current_state(line):
    gcode, x, y, z, feed_rate, extrusion, mcode = extract_gcode(line)

    if x is not None:
        current_x = x
    if y is not None:
        current_y = y
    if extruder_on:
        current_z = 0  # Set Z to 0 when the extruder is on
    else:
        current_z = 5  # Set Z to 5 when the extruder is off
    if feed_rate is not None:
        current_feed_rate = feed_rate
    if extrusion is not None:
        current_extrusion = extrusion
    if mcode is not None:
        if mcode == "M101":
            extruder_on = True
        elif mcode == "M103":
            extruder_on = False

    return gcode, current_x, current_y, current_z, current_feed_rate, current_extrusion, extruder_on

def extract_gcode(line):
    gcode = None
    x, y, z, feed_rate, extrusion, mcode = None, None, None, None, None, None
    words = line.split()

    for word in words:
        if word.startswith("G"):
            gcode = word
        elif word.startswith("X"):
            x = word[1:]
        elif word.startswith("Y"):
            y = word[1:]
        elif word.startswith("Z"):
            z = word[1:]
        elif word.startswith("F"):
            feed_rate = word[1:]
        elif word.startswith("E"):
            extrusion = word[1:]
        elif word.startswith("M"):
            mcode = word

    return gcode, x, y, z, feed_rate, extrusion, mcode

def current_position_callback(msg):
    # global current_x, current_y, current_z
    current_x, current_y, current_z = msg.data[0], msg.data[1], msg.data[2]

def main():
    rospy.init_node("gcode_parser_node")
    pub = rospy.Publisher("gcode_info", GCodeInfo, queue_size=10)

    # Subscribe to the /currentPosition topic
    rospy.Subscriber("/currentPosition", Float32MultiArray, current_position_callback)

    while not rospy.is_shutdown():
        # Check if the /currentPosition has been updated
            with open("your_gcode_file.gcode", "r") as file:
                for line in file:
                    if current_x is not None and current_y is not None and current_z is not None:
                        gcode, x, y, z, feed_rate, extrusion, extruder_on = update_current_state(line)

                        if (gcode or x or y or z or feed_rate or extrusion or extruder_on):
                            gcode_info = GCodeInfo()
                            gcode_info.gcode = gcode
                            gcode_info.x = x
                            gcode_info.y = y
                            gcode_info.z = z
                            gcode_info.feed_rate = feed_rate
                            gcode_info.extruder_on = extruder_on

                            pub.publish(gcode_info)                    

                    # Reset current_x, current_y, and current_z after publishing
                    current_x, current_y, current_z = None, None, None

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass