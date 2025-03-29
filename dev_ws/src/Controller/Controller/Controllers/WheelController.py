
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

"""
Controller for the wheels
It will subscribe to topics regarding the wheels
States that need to operate wheels, get a reference to this class.
They can call methods on this class, which will transmit to a topic.
"""
class WheelController(Node):

    def __init__(self):

        super().__init__("WheelController")
        
        self.publisher = self.create_publisher(
            String,
            'moveWheels',
            10
        )
        
    """
    Move robot forward
    """
    def move_forward(self):
        self.__publish_string("moveForward")

    """
    Move robot backwards
    """
    def move_backwards(self):
        self.__publish_string("moveBackwards")

    
    """
    Rotate robot clockwise
    """
    def rotate_clockwise(self):
        self.__publish_string("RotateCW")

    """
    Stop the robot
    """
    def stop(self):
        self.__publish_string("stop")

    """
    Private method, do not call
    """
    def __publish_string(self, string):
        msg = String()
        msg.data = string
        self.publisher.publish(msg)



