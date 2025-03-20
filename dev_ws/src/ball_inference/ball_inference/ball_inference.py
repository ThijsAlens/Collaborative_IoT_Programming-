
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

from ament_index_python.packages import get_package_share_directory
import cv2
import os
import random


def main(args = None):

    rclpy.init(args=args)
    publisher = BallInference()

    rclpy.spin(publisher)
    
    publisher.destroy_node()
    rclpy.shutdown()

class BallInference(Node):
    
    def __init__(self):
        super().__init__("BallInference")

        self.publisher = self.create_publisher(Bool, 'ballInference', 10)

        print("Created BallInference node")

        self.subscription = self.create_subscription(
            Image, 
            "getImage",
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        ball_detected = self.process_image(msg)

        output_msg = Bool()
        output_msg = ball_detected
        self.get_logger().info("Calling inference; detected ball = " + ball_detected)
        self.publisher.publish(output_msg)


    def process_image(self,image):
    
        # Convert image
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

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
            if 500 < area < 50000:
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

        return found_ball









