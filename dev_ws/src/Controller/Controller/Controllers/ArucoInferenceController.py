
import rclpy
from rclpy.node import Node
from custom_msg_aruco.msg import PositionStatus

"""
Controller for the ball inference
It will subscribe to topics regarding the Ball inference
They can call methods on this class, which will transmit to a topic.
"""
class ArucoInferenceController(Node):


    def __init__(self):

        super().__init__("ArucoController")

        self.aruco_found: bool = False
        self.aruco_position: tuple[float, float]

        self.create_subscription(
            PositionStatus,
            "arucoDetection",
            self.__InferenceCallback,
            10
        )

    """
    Private method, do not call
    """
    def __InferenceCallback(self, msg):
        self.aruco_found = msg.found
        self.position = (msg.x, msg.y)



