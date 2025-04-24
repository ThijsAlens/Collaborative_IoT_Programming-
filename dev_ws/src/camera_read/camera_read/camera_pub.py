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

        self.capture = cv2.VideoCapture(0, cv2.CAP_V4L2)

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.capture.set(cv2.CAP_PROP_FPS, 30)

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.capture.read()

        if ret == True:
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame))

            self.get_logger().info('Publishing video frame')
        else:
            self.get_logger().info("couldn't get a frame")



def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)

    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
