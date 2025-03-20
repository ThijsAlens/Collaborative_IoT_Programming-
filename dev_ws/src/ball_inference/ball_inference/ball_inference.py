
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import random

def main(args = None):
    rclpy.init(args=args)
    publisher = IntPublisher()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()

class BallInference(Node):
    
    def __init__(self, image):
        super().__init__("BallInference")

        self.publisher = self.create_publisher(Bool, 'ballInference', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def timer_callback(self):

        msg = Int64()
        msg.data = random.randint(0, 4)

        self.publisher.publish(msg)
        self.get_logger().info('Publishing ' & msg.data)


    def process_image(self, image):
        





