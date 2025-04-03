import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class ClampPublisher(Node):
    def __init__(self):
        # Super calls the Node class constructor
        super().__init__('clamp_publisher') # Argument is name of node
        # Declares that node publishes messages of type String, over a topic
        # named topic and the queue size is 10 (QoS)
        self.publisher_ = self.create_publisher(String, 'clamp', 10)
        timer_period = 2 #seconds
        self.timer = self.create_timer(timer_period, self.send_command)
        self.state = 'open'
    def send_command(self):
        """ Creates a message with the counter value appended and
        publishes it to the console with the get_logger().info
        function.
        """
        msg = String()
        if self.state == 'open':
            msg.data = 'close'
            self.state = 'close'
        else:
            msg.data = 'open'
            self.state = 'open'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing data: "{msg.data}"')
    
def main(args=None):
    """ Initializes rclpy, then creates the node. Then spins the node
    so its callbacks are called.
    """
    rclpy.init(args=args)
    clamp_publisher = ClampPublisher()
    rclpy.spin(clamp_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    clamp_publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()