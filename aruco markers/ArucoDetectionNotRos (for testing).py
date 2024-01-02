import cv2

class ArucoMarkerDetector:
    def __init__(self, aruco_dict_type="DICT_4X4_250"):
        self.aruco_dict_type = aruco_dict_type
        self.cap = self.initialize_camera()
        self.marker_info = None
        self.ARUCO_DICT = {"DICT_4X4_250": cv2.aruco.DICT_4X4_250,}

    def initialize_camera(self):
        stream_addr = 'http://192.168.0.20/video.cgi'
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        return cap

    def detect_markers(self, image):
        arucoDict = cv2.aruco.getPredefinedDictionary(self.ARUCO_DICT[self.aruco_dict_type])
        arucoParams = cv2.aruco.DetectorParameters()
        
        corners, ids, _ = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

        if ids is not None and len(ids) == 4:
            return corners, ids
        else:
            return None, None

    def process_video_capture(self):
        while self.cap.isOpened():
            ret, img = self.cap.read()

            h, w, _ = img.shape
            width = 1000
            height = int(width * (h / w))
            img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)

            corners, ids = self.detect_markers(img)

            if corners is not None and ids is not None:
                self.marker_info = {}
                marker_corners_dict = []

                for (markerCorner, markerID) in zip(corners, ids):
                    corners = markerCorner.reshape((4, 2))
                    tLeft = (int(corners[0][0]), int(corners[0][1]))
                    
                    marker_corners_dict.append(tLeft)

                    self.marker_info[markerID[0]] = {
                        "top_left_corner": tLeft
                    }

                    cv2.line(img, tLeft, (int(corners[1][0]), int(corners[1][1])), (0, 255, 0), 2)
                    cv2.line(img, (int(corners[1][0]), int(corners[1][1])),
                             (int(corners[2][0]), int(corners[2][1])), (0, 255, 0), 2)
                    cv2.line(img, (int(corners[2][0]), int(corners[2][1])),
                             (int(corners[3][0]), int(corners[3][1])), (0, 255, 0), 2)
                    cv2.line(img, (int(corners[3][0]), int(corners[3][1])), tLeft, (0, 255, 0), 2)

                    cv2.circle(img, tLeft, 4, (0, 0, 255), -1)

                    cv2.putText(img, str(markerID[0]), (tLeft[0], tLeft[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    if len(marker_corners_dict) == 4:
                        for i in range(4):
                            cv2.line(img, marker_corners_dict[i], marker_corners_dict[(i + 1) % 4], (255, 0, 0), 2)

            cv2.imshow("Image", img)


            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        cv2.destroyAllWindows()
        self.cap.release()

    def get_marker_info(self):
        return self.marker_info

if __name__ == "__main__":
    aruco_detector = ArucoMarkerDetector(aruco_dict_type="DICT_4X4_250")
    aruco_detector.process_video_capture()
    marker_info = aruco_detector.get_marker_info()
    print("Marker Info:", marker_info)