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

    def __init__(self):

        super().__init__("UltrasoundInferenceController")

        self.distance = 0
        self.speed = 0

        self.subscription = self.create_subscription(
            Int32MultiArray, 
            "read_ultrasound_sensor",
            self.__InferenceCallback,
            10
        )

        #self._publish_number()


    """
    Private method, do not call
    """
    def __InferenceCallback(self, msg):
        self.distance = msg.data[0] 
        self.speed = msg.data[1] 

