#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32MultiArray
import argparse

# Define variables to store the current state of coordinates, feed rate, and extruder status.
current_x, current_y, current_z = 1, 1, 1

def update_current_state(line):
    gcode, x, y, z, feed_rate, extruder_on, mcode = extract_gcode(line)
    current_feed_rate, current_extrusion = None, None
    current_x1, current_y1, current_z1 = None, None, None
    if x is not None:
        current_x1 = x
    else:
        current_x1 = 0
    if y is not None:
        current_y1 = y
    else:
        current_y1 = 0
    if extruder_on:
        current_z1 = 0  # Set Z to 0 when the extruder is on
    else:
        current_z1 = 5  # Set Z to 5 when the extruder is off
    if feed_rate is not None:
        current_feed_rate = feed_rate
    if extruder_on is not None:
        current_extrusion = extruder_on
    if mcode is not None:
        if mcode == "M101":
            extruder_on = True
        elif mcode == "M103":
            extruder_on = False

    return gcode, current_x1, current_y1, current_z1, current_feed_rate, current_extrusion, extruder_on

def extract_gcode(line):
    gcode = None
    x, y, z, feed_rate, extrusion, mcode = None, None, None, None, None, None
    words = line.split()

    for word in words:
        if word.startswith("G"):
            gcode = word
        elif word.startswith("X"):
            x = word[1:]
            x = float(word[1:])
        elif word.startswith("Y"):
            y = word[1:]
            y = float(word[1:])
        elif word.startswith("Z"):
            z = word[1:]
            z = float(word[1:])
        elif word.startswith("F"):
            feed_rate = word[1:]
            feed_rate = float(word[1:])
        elif word.startswith("E"):
            extrusion = word[1:]
            extrusion = float(word[1:])
        elif word.startswith("M"):
            mcode = word

    return gcode, x, y, z, feed_rate, extrusion, mcode

def current_position_callback(msg):
    global current_x, current_y, current_z
    current_x, current_y, current_z = msg.data[0], msg.data[1], msg.data[2]
    print(current_x)

def main():
    global current_x, current_y, current_z, current_feed_rate, current_extrusion, extruder_on
    current_feed_rate, current_extrusion, extruder_on = None, None, False
    current_x, current_y, current_z = 1, 1, 1
    rospy.init_node("gcode_parser_node", anonymous=True)
    position_pub = rospy.Publisher('/gcode_position', Float32MultiArray, queue_size=10)

    parser = argparse.ArgumentParser(description="Extract and interpret G-code information from a G-code file.")
    parser.add_argument("gcode_file", help="Path to the G-code file")
    parser.add_argument("--output", type=int, default=1, choices=[0, 1], help="Enable (1) or disable (0) output")
    args = parser.parse_args()

    # Subscribe to the /currentPosition topic
    rospy.Subscriber("/currentPosition", Float32MultiArray, current_position_callback)

    while not rospy.is_shutdown():
        # Check if the /currentPosition has been updated
            with open(args.gcode_file, "r") as file:
                for line in file:
                    if current_x is not None and current_y is not None and current_z is not None:
                        gcode, x, y, z, feed_rate, extrusion, extruder_on = update_current_state(line)

                        if args.output == 1 and (gcode or x or y or z or feed_rate or extrusion or extruder_on):
                            position_data = [x, y, z]
                            position_msg = Float32MultiArray(data=position_data)
                            position_pub.publish(position_msg)
                            
                            output_line = ["G-code: {}".format(gcode), "X: {}".format(x), "Y: {}".format(y), "Z: {}".format(z), "F: {}".format(feed_rate)]
                            
                            if extruder_on:
                                output_line.append("Extruder: On")
                            else:
                                output_line.append("Extruder: Off")
                            
                            gcode_str = ", ".join(output_line)
                            rospy.loginfo(gcode_str)

		            # Reset current_x, current_y, and current_z after publishing
		            current_x, current_y, current_z = None, None, None

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
