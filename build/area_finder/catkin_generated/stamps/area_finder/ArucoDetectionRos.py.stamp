#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ArucoMarkerDetectorROS:
    def __init__(self, aruco_dict_type="DICT_4X4_250"):
        self.aruco_dict_type = aruco_dict_type
        self.cap = self.initialize_camera()
        self.bridge = CvBridge()
        self.offset_pub = rospy.Publisher('/offset', Float32MultiArray, queue_size=1)
        #self.marker_info_pub = rospy.Publisher('aruco_marker_info', Int32MultiArray, queue_size=10)
        self.ARUCO_DICT = {"DICT_4X4_250": cv2.aruco.DICT_4X4_250}

        # Define the width of the object in the real world (in mm)
        self.real_world_width_mm = 100

        # Define the width of the object in pixels in the original image
        self.object_width_pixels = 136

        # Define the robot's original position (assuming the robot is at (500, 20) in the original image)
        #self.robot_x_original = 500
        #self.robot_y_original = 20

        # Subscribe to the '/readyForOffset' topic
        rospy.Subscriber('/readyForOffset', Bool, self.ready_for_offset_callback)

    def ready_for_offset_callback(self, msg):
        # Callback function for '/readyForOffset'
        #rospy.loginfo(rospy.get_caller_id() + "Received readyForOffset message")
        # Call the process_video_capture function when readyForOffset message is received
        self.cap = self.initialize_camera()

        self.process_video_capture()
        #rospy.loginfo(rospy.get_caller_id() + "Should be sent")

    def initialize_camera(self):
        stream_addr = 'http://192.168.0.20/video.cgi'
        cap = cv2.VideoCapture(stream_addr)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        return cap

    def detect_markers(self, image):
        arucoDict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[self.aruco_dict_type])
        #arucoParams = cv2.aruco.DetectorParameters()
        arucoParams = cv2.aruco.DetectorParameters_create()
        
        corners, ids, _ = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

        if ids is not None:
            return corners, ids
        else:
            return None, None

    def process_video_capture(self):
        #while not rospy.is_shutdown() and self.cap.isOpened():
        while not rospy.is_shutdown():
            ret, img = self.cap.read()

            if not ret or img is None:
                #print "Failed to read frame from camera."
                continue

            #print "Original image dimensions:", img.shape
            h, w, _ = img.shape
            if h > 0 and w > 0:
                width = 1000
                height = int(width * (float(h) / w))  # Ensure floating-point division
                img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)

            # Calculate the scaling factor
            scaling_factor = self.real_world_width_mm / float(self.object_width_pixels)
            #print "Scaling factor:", scaling_factor

            # Resize the image based on the scaling factor
            img = cv2.resize(img, None, fx=scaling_factor, fy=scaling_factor, interpolation=cv2.INTER_CUBIC)
            #print "Resized image dimensions:", img.shape

            """ 
            h, w, _ = img.shape
            if h > 0 and w > 0:
                width = 1000
                height = int(width * (float(h) / w))  # Ensure floating-point division
                img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)"""

            # Calculate the scaling factor
            #scaling_factor = self.real_world_width_mm / self.object_width_pixels

            # Resize the image based on the scaling factor
           # img = cv2.resize(img, None, fx=scaling_factor, fy=scaling_factor, interpolation=cv2.INTER_CUBIC)
            
            corners, ids = self.detect_markers(img)

            if corners is not None and ids is not None:
                marker_info = []
                marker_corners_dict = []

                for (markerCorner, markerID) in zip(corners, ids):
                    corners = markerCorner.reshape((4, 2))
                    tLeft = (int(corners[0][0]), int(corners[0][1]))
                    
                    # Calculate the relative position of the ArUco marker with respect to the robot
                    robot_x_original = 391
                    robot_y_original = 414
                    relative_y = -(tLeft[0] - robot_x_original)
                    relative_x = -(tLeft[1] - robot_y_original)

                    marker_corners_dict.append(tLeft)

                    #marker_info.append(markerID[0])

                    marker_info.append((markerID[0], relative_y, relative_x))

                    cv2.line(img, tLeft, (int(corners[1][0]), int(corners[1][1])), (0, 255, 0), 2)
                    cv2.line(img, (int(corners[1][0]), int(corners[1][1])),
                             (int(corners[2][0]), int(corners[2][1])), (0, 255, 0), 2)
                    cv2.line(img, (int(corners[2][0]), int(corners[2][1])),
                             (int(corners[3][0]), int(corners[3][1])), (0, 255, 0), 2)
                    cv2.line(img, (int(corners[3][0]), int(corners[3][1])), tLeft, (0, 255, 0), 2)

                    cv2.circle(img, tLeft, 4, (0, 0, 255), -1)

                    cv2.putText(img, str(markerID[0]), (tLeft[0], tLeft[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    
                    """                     if len(marker_corners_dict) == 4:
                    for i in range(4):
                        cv2.line(img, marker_corners_dict[i], marker_corners_dict[(i + 1) % 4], (255, 0, 0), 2) """

                    # Sort marker_info by marker ID
                    marker_info.sort(key=lambda x: x[0])

                    # Create the marker_info_msg
                    #marker_info_msg = Int32MultiArray(data=[item for info in marker_info for item in info])

                    # Publish marker information
                    #self.marker_info_pub.publish(marker_info_msg)
                    
                    bottom_left_marker = min(marker_info, key=lambda info: (info[1], info[2]))
                    offset_msg = Float32MultiArray(data=[bottom_left_marker[2], bottom_left_marker[1]])
                    rospy.loginfo(rospy.get_caller_id() + "I heard X=%f and Y=%f", offset_msg.data[0],offset_msg.data[1])
                    self.offset_pub.publish(offset_msg)
                    cv2.destroyAllWindows()
                    self.cap.release()
                    return
        """ 
            cv2.imshow("Image", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        cv2.destroyAllWindows()
        self.cap.release() """

    def publish_bottom_left_marker(self, marker_info):
        # Find the marker with the lowest X and Y values
        if marker_info:
            bottom_left_marker = min(marker_info, key=lambda info: (info[1], info[2]))
            offset_msg = Float32MultiArray(data=[bottom_left_marker[1], bottom_left_marker[2]])
            self.offset_pub.publish(offset_msg)

    def get_perspective_transform(self, marker_info, g_code_list):
        # Extract corners from marker_info (assuming they are in the correct order)
        src_pts = np.array([info[1:3] for info in marker_info], dtype="float32")

        # Calculate the bounding box of the G-code list with an added margin
        margin = 50  # Margin of 50 units
        min_x = min(point[0] for point in g_code_list) - margin
        max_x = max(point[0] for point in g_code_list) + margin
        min_y = min(point[1] for point in g_code_list) - margin
        max_y = max(point[1] for point in g_code_list) + margin

        # Destination points based on the G-code list's bounding box with margin
        dst_pts = np.array([[min_x, min_y], [max_x, min_y], [max_x, max_y], [min_x, max_y]], dtype="float32")

        # Compute the perspective transform matrix
        matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
        return matrix


    def scale_point(self, point, matrix):
        # Apply the perspective transform only to the X and Y coordinates
        scaled_xy = np.dot(matrix, [point[0], point[1], 1])

        # Normalize the scaled point
        scaled_xy = scaled_xy / scaled_xy[2]

        # Return the scaled X and Y coordinates along with the original Z coordinate
        return (scaled_xy[0], scaled_xy[1], point[2])

    def scale_g_code_list(self, g_code_list, matrix):
        scaled_list = []
        for point in g_code_list:
            # Ensure that 'point' includes the Z coordinate
            scaled_point = self.scale_point(point, matrix)
            scaled_list.append(scaled_point)
        return scaled_list

if __name__ == "__main__":
    rospy.init_node('aruco_marker_detector', anonymous=True)
    aruco_detector = ArucoMarkerDetectorROS(aruco_dict_type="DICT_4X4_250")
    aruco_detector.process_video_capture()
    rospy.spin()
