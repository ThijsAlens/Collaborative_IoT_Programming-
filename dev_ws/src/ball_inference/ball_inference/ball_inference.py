
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

from ament_index_python.packages import get_package_share_directory
import cv2
import os
import random


def main(args = None):

    package_path = get_package_share_directory('ball_inference')

    dir_path = "Collaborative_IoT_Programming-/dev_ws/install/ball_inference/share/ball_inference/data/"
    

    dir_path = os.path.join(package_path, "data")

    print(dir_path + "image1.png")
    image1 = cv2.imread(dir_path + "image1.png")
    image2 = cv2.imread(dir_path + "image2.png")
    image3 = cv2.imread(dir_path + "image3.png")
    print(image1)

    rclpy.init(args=args)
    publisher = BallInference(image1, image2, image3)

    #rclpy.spin(publisher)

    #publisher.destroy_node()
    #rclpy.shutdown()

class BallInference(Node):
    
    def __init__(self, image1, image2, image3):
        super().__init__("BallInference")

        self.publisher = self.create_publisher(Bool, 'ballInference', 10)
        #timer_period = 0.5
        #self.timer = self.create_timer(timer_period, self.timer_callback)

        ball_detected = self.process_image(image1)
        print(ball_detected)

        ball_detected = self.process_image(image2)
        print(ball_detected)

        ball_detected = self.process_image(image3)
        print(ball_detected)


    def timer_callback(self):

        msg = Int64()
        msg.data = random.randint(0, 4)

        self.publisher.publish(msg)
        self.get_logger().info('Publishing ' & msg.data)


    def process_image(self, image):
        
          # Convert image
        #cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')

        cv_image = image

        # Blur to reduce noise
        blurred = cv2.GaussianBlur(cv_image, (11, 11), 0)

        # Convert to HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Define color range for tennis ball (yellow-green)
        lower_yellow = np.array([163, 162, 0])
        upper_yellow = np.array([255, 229, 0])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Morphological operations to clean up the mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        avg = sum(mask)/len(mask)

        return avg > 0.25









