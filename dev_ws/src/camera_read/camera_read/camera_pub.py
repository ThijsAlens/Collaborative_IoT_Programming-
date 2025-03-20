import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):

        super().__init__('ImagePublisher')
        self.publisher_ = self.create_publisher(Image, 'getImage', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.capture = cv2.VideoCapture(0)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.capture.read()

        if ret == True:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            self.get_logger().info("len: ", len(frame))

            self.get_logger().info('Publishing video frame')


def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)

    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
