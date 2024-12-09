#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from ros2_aruco_interfaces.msg import ArucoMarkers
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv2
import math
# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def draw_circle_over_aruco(image):
  for aruco_dict_name in ARUCO_DICT.keys():
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dict_name])



    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(image, aruco_dict,
      parameters=arucoParams)
    if (ids):
      my_corners = corners
      print(ids)

  # print(image.size)
  print(my_corners)
  center = [0, 0]
  for corner_coordinates in my_corners[0][0]:
    x = corner_coordinates[0]
    y = corner_coordinates[1]

    center[0] += x
    center[1] += y 

  center[0] = center[0]//4
  center[1] = center[1]//4

  my_corners = my_corners[0][0]
  r = ((center[0] - my_corners[0][0])**2 + (center[1] - my_corners[0][1])**2)**0.5
  image = cv2.circle(image,(int(center[0]), int(center[1])), radius=math.ceil(r), color=(0, 0, 255), thickness=5)
  return image

class PrintNode(Node):
    def __init__(self):
        self.bridge = CvBridge()
        self.first_marker_id = 11
        self.current_marker = None
        self.lastSeenImage = None
        super().__init__('acuro_marker_detector')
        self.get_logger().info('acuro_marker_detector has been started.')
        self.arucoMarkersSubscriber = self.create_subscription(ArucoMarkers, '/aruco_markers', self.ArucoMarkersCallback, 10)
        self.cameraImageSubscriber = self.create_subscription(Image, '/camera/image_raw', self.CameraImageCallback, 10)
        self.imagePublisher = self.create_publisher(Image, 'detected_aruco', 10)
    
    def ArucoMarkersCallback(self, msg):
        '''
        This is the markers callback function. It is called whenever we see a marker on the camera feed.
        '''
        if (len(msg.marker_ids)>0):
            marker_id = msg.marker_ids[0]
            if (self.isNextMarker(marker_id)):
                self.doMarkerOperations(marker_id)
                
    def CameraImageCallback(self, msg):
        self.lastSeenImage = msg

    def isNextMarker(self, id):
        if self.current_marker is None and self.first_marker_id == id:
            return True
        elif self.current_marker is not None and id == self.current_marker+1:
            return True
        return False

    def doMarkerOperations(self, id):
        self.get_logger().info('Perform operations on for marker: '+str(id))
        cv2FormatImage = self.bridge.imgmsg_to_cv2(self.lastSeenImage, desired_encoding='bgr8')
        processedImage = draw_circle_over_aruco(cv2FormatImage)
        processed_image_msg = self.bridge.cv2_to_imgmsg(processedImage, encoding='bgr8')
        self.imagePublisher.publish(processed_image_msg)
        
        self.current_marker = id

def main(args=None):
    rclpy.init(args=args)
    node = PrintNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()