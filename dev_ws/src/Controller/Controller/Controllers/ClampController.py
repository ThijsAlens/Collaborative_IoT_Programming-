
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

"""
Controller for the clamp
It will subscribe to topics regarding the clamp
"""
class ClampController(Node):

    def __init__(self):

        super().__init__("ClampController")
        
        self.publisher = self.create_publisher(
            String,
            'clamp',
            10
        )
        
    """
    Open clamp (unsqueeze)
    """
    def open_clamp(self):
        self._publish_string("open")
        
    """
    Close clamp (squeeze)
    """
    def close_clamp(self):
        self._publish_string("close")

    """
    Private method, do not call
    """
    def _publish_string(self, string):
        msg = String()
        msg.data = string
        self.publisher.publish(msg)