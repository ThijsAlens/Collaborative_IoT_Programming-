import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory

import cv2
import numpy as np
import random
from custom_msg_aruco.msg import PositionStatus
from cv_bridge import CvBridge

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.subscriptionAruco = self.create_subscription(
            Image, 
            "getImage",
            self.listener_callback_aruco,
            10
        )
        
        self.subscriptionBall = self.create_subscription(
            Image, 
            "getImage",
            self.listener_callback_ball,
            10
        )

        self.publisherArucoDetection = self.create_publisher(PositionStatus, 'arucoDetection', 10)

        #self.create_timer(0.25, self.tijdelijk)
  
        self.publisherBallInference = self.create_publisher(PositionStatus, 'ballInference', 10)

        print("Created Aruco + BallInference node")

        self.bridge = CvBridge()

        # # Define the dictionary and parameters for ArUco marker detection
        # self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        # self.parameters = cv2.aruco.DetectorParameters_create()  # Updated line  
    
    def listener_callback_aruco(self, msg):
        output_msg = PositionStatus()
        output_msg.found, center = self.process_image_aruco(msg)
        output_msg.x, output_msg.y = center
        self.get_logger().info("from ARUCO DETECTOR, Sending: found = " + str(output_msg.found) + " | x = " + str(output_msg.x) + " | y = " + str(output_msg.y))

        self.publisherArucoDetection.publish(output_msg)
     
    def listener_callback_ball(self, msg):

        output_msg = PositionStatus()
        output_msg.found, center = self.process_image_ball(msg)
        output_msg.x, output_msg.y = center
        self.get_logger().info("Calling inference; detected ball = " + str(output_msg.found) + " at " + str(output_msg.x) + " " + str(output_msg.y))
        self.publisherBallInference.publish(output_msg)

    def process_image_aruco(self, image):
        try:
            # Load the image (you can change this to an encoded image if you prefer)
            cv_image = self.bridge.imgmsg_to_cv2(image)
            
            # cv2.imshow("camera", cv_image)
            # cv2.waitKey(1)

            # Resize image to 640x480 (optional)
            #cv_image_resized = cv2.resize(cv_image, (640, 480))
            # Detect ArUco markers
            #detected, position = self.detect_ARUCO(cv_image)
            return self.detect_ARUCO(cv_image)

            # Log the result of the detection
            #self.get_logger().info(f"Detection Result: {detected}")
            #return detected, position

        except Exception as e:
            self.get_logger().error(f"Error in processing image: {e}")
    
    def process_image_ball(self,image):
    
        # Convert image
        cv_image = self.bridge.imgmsg_to_cv2(image)

        # cv2.imshow("camera", cv_image)
        # cv2.waitKey(1)

        # Blur to reduce noise
        blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)

        # Convert to HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Define color range for tennis ball (yellow-green)
        lower_yellow = np.array([28, 100, 100])
        upper_yellow = np.array([38, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Morphological operations to clean up the mask
        mask = cv2.erode(mask, None, iterations=3)
        mask = cv2.dilate(mask, None, iterations=3)


        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        found_ball = False    

        masks_circles = np.zeros_like(mask)
        for contour in contours:

            area = cv2.contourArea(contour)

            # Filter out small or excessively large shapes
            if 200 < area < 50000:
                # Fit a circle around the contour
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                circle_area = np.pi * (radius ** 2)

                # Calculate pixel density inside the circle
                mask_circle = np.zeros_like(mask)
                mask_visualisation = np.zeros_like(mask)
                cv2.circle(mask_circle, center, radius, 255, -1)
                cv2.circle(mask_visualisation, center, radius, 255, 3)
                points_inside = cv2.countNonZero(mask & mask_circle)
                density = points_inside / circle_area
                if(density > 0.6):
                    masks_circles = masks_circles + mask_visualisation
                    found_ball = True
                if(found_ball):
                    return found_ball, center

        return False, (0,0)

    def detect_ARUCO(self, image: np.ndarray): # -> tuple(bool, tuple(int, int)):
        """
        Function to detect ARUCO markers in an image
        
        Parameters
            image (np.ndarray): The image in which to detect the ARUCO markers
        
        Returns
            tuple[bool, tuple[int, int]]: A tuple containing a boolean indicating if an ARUCO marker was detected and the center of the detected marker
        """
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Define the dictionary for the ARUCO markers
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        
        # Set the detector parameters
        parameters = cv2.aruco.DetectorParameters_create()
        
        # Detect the markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # Check if any markers were detected
        if ids is not None:
            for i, id in enumerate(ids):
                if id == 1:  # Assuming you are looking for the marker with ID 1
                    corner_points = corners[i][0]  # (4,2) array -> (x, y) coordinates of the 4 corners
                    center_x = int(np.mean(corner_points[:, 0]))
                    center_y = int(np.mean(corner_points[:, 1]))
                    center = (center_x, center_y)
                    return True, center
        return False, (0, 0)



def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()

    rclpy.spin(aruco_detector)
    rclpy.spin(publisher)
    publisher = BallInference()

    publisher.destroy_node()
    aruco_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
