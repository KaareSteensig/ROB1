#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Float32MultiArray, Empty
import numpy as np

class RoutePlanner:
    def __init__(self):
        rospy.init_node('routePlanner')
        
        self.current_position = None
        self.goal_position = None
        self.distance_between_points = 10  # Adjust this value as needed

        self.currentPosition_published = False  # Flag to track if currentPosition has been published

        self.goal_reached = False
        self.path = []
        self.path_idx = 0

        # Subscribers
        rospy.Subscriber('/goalPosition', Float32MultiArray, self.goal_position_callback)
        rospy.Subscriber('/currentPosition', Float32MultiArray, self.current_position_callback)
        rospy.Subscriber('/pathpointreached', Empty, self.path_point_reached_callback)

        # Publisher
        self.position_publisher = rospy.Publisher('/position', Float32MultiArray, queue_size=10)
        self.end_point_publisher = rospy.Publisher('/endPointReached', Bool, queue_size=1)
        # self.current_position_publisher = rospy.Publisher('/currentPosition', Float32MultiArray, queue_size=10)


    def goal_position_callback(self, data):
        self.goal_position = data.data
        rospy.loginfo(rospy.get_caller_id() + " goal position: X=%f and Y=%f and Z=%f", data.data[0],data.data[1],data.data[2])
        self.calculate_path()

    def current_position_callback(self, data):
        self.current_position = data.data
        rospy.loginfo(rospy.get_caller_id() + " current position: X=%f and Y=%f and Z=%f", data.data[0],data.data[1],data.data[2])

    def path_point_reached_callback(self, data):
        self.publish_next_position()
        rospy.loginfo(rospy.get_caller_id() + " goal reached!")
        
        #self.goal_reached = true
        #rospy.loginfo(rospy.get_caller_id() + " goal reached: %s", str(data.data))
        #if self.goal_reached:
        #    self.publish_next_position()

    def calculate_path(self):
        if self.current_position and self.goal_position:
            direction_vector = np.array(self.goal_position) - np.array(self.current_position)
            total_distance = np.linalg.norm(direction_vector)
            
            num_points = int(total_distance / self.distance_between_points)
            if num_points < 2:
                self.path = [self.current_position, self.goal_position]
                
            else:
                #self.currentPosition_published = False  # Flag to track if currentPosition has been published
                path = []
                for i in range(num_points):
                    t = float(i) / float(num_points - 1)  # Correct interpolation parameter
                    x = self.current_position[0] + t * direction_vector[0]
                    y = self.current_position[1] + t * direction_vector[1]
                    z = self.current_position[2] + t * direction_vector[2]
                    path.append([x, y, z])
                    # Debug print statements
                    #rospy.loginfo("Debug: i=%d, t=%.2f, x=%.2f, y=%.2f, z=%.2f", i, t, x, y, z)
                
                path.append(self.goal_position)
                self.path = path

            # Publish the first point in the path
            self.path_idx = 0
            self.publish_next_position()

            # Log the calculated path
            #rospy.loginfo("Calculated Path:")
            #for point in self.path:
                #rospy.loginfo("Point: x=%.2f, y=%.2f, z=%.2f", point[0], point[1], point[2])

    def publish_next_position(self):
        if self.path_idx < len(self.path):
            next_position = self.path[self.path_idx]
            self.path_idx += 1
            #rospy.loginfo("Publishing next position")

            msg = Float32MultiArray(data=next_position)
            self.position_publisher.publish(msg)
        else:
            #publish til end point reached
            msg = True
            self.end_point_publisher.publish(msg)

    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.goal_reached:
                self.publish_next_position()
            #if self.path and not self.currentPosition_published:
                #rospy.loginfo("path found")
                # If there is a path, publish the last point as the current position
                #if self.path_idx >= len(self.path) and not self.currentPosition_published:
                    #rospy.loginfo("publishing to current position")
                    #current_position_msg = Float32MultiArray(data=self.path[-1])
                    #self.current_position_publisher.publish(current_position_msg)
                    #self.currentPosition_published = True  # Set the flag to True
            rate.sleep()

if __name__ == '__main__':
    route_planner = RoutePlanner()
    route_planner.run()

