import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from custom_msg_aruco.msg import PositionStatus
from cv_bridge import CvBridge

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.subscription = self.create_subscription(
            Image, 
            "getImage",
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(PositionStatus, 'arucoDetection', 10)

        self.bridge = CvBridge()

        # # Define the dictionary and parameters for ArUco marker detection
        # self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        # self.parameters = cv2.aruco.DetectorParameters_create()  # Updated line  
    
    def listener_callback(self, msg):
        output_msg = PositionStatus()
        output_msg.found, output_msg.x, output_msg.y = self.process_image(msg)
        self.get_logger().info("from ARUCO DETECTOR, Sending: found = " + str(output_msg.found) + " | x = " + str(output_msg.x) + " | y = " + str(output_msg.y))

        self.publisher.publish(output_msg)

    def process_image(self, image):
        try:
            # Load the image (you can change this to an encoded image if you prefer)
            cv_image = self.bridge.imgmsg_to_cv2(image)
            
            cv2.imshow("camera", cv_image)
            cv2.waitKey(1)

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

    def detect_ARUCO(self, image: np.ndarray) -> tuple[bool, tuple[int, int]]:
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
    # Cleanup
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
