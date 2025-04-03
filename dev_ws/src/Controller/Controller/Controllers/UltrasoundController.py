import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32

"""
Controller for the ultrasound inference
It will subscribe to topics regarding the ultrasound inference
They can call methods on this class, which will transmit to a topic.
"""
class UltrasoundInferenceController(Node):

#nog aan te werken
    def __init__(self):

        super().__init__("UltrasoundInferenceController")


        self.publisher = self.create_publisher(
            Int32MulitArray, 
            "read_ultrasound_sensor",
            10
        )

    """
    Distance from US
    """
    def distance(self):
        self._publish_number()


    """
    Private method, do not call
    """
    def _publish_number(self, number):
        msg = Int32()
        msg.data = number
        self.publisher.publish(msg)
