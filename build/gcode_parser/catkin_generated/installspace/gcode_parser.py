#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
import argparse

# Define variables to store the current state of coordinates, feed rate, and extruder status.
current_x, current_y, current_z, current_feed_rate, current_extrusion = None, None, None, None, None
extruder_on = False  # Assume extruder is initially off

def update_current_state(line):
    global current_x, current_y, current_z, current_feed_rate, current_extrusion, extruder_on
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

def main():
    rospy.init_node('gcode_parser_node', anonymous=True)
    gcode_pub = rospy.Publisher('gcode', String, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    
    parser = argparse.ArgumentParser(description="Extract and interpret G-code information from a G-code file.")
    parser.add_argument("gcode_file", help="Path to the G-code file")
    parser.add_argument("--output", type=int, default=1, choices=[0, 1], help="Enable (1) or disable (0) output")
    args = parser.parse_args()

    with open(args.gcode_file, "r") as file:
        for line in file:
            gcode, x, y, z, feed_rate, extrusion, extruder_on = update_current_state(line)

            if args.output == 1 and (gcode or x or y or z or feed_rate or extrusion or extruder_on):
                output_line = ["G-code: {}".format(gcode), "X: {}".format(x), "Y: {}".format(y), "Z: {}".format(z), "F: {}".format(feed_rate)]

                if extruder_on:
                    output_line.append("Extruder: On")
                else:
                    output_line.append("Extruder: Off")

                gcode_str = ", ".join(output_line)
                rospy.loginfo(gcode_str)
                gcode_pub.publish(gcode_str)
                
                rate.sleep()

if __name__ == "__main__":
    main()
