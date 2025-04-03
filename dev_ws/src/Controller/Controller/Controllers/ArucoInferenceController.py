
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

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
            Bool,
            "aruco_detection",
            self.__InferenceCallback,
            10
        )

    """
    Private method, do not call
    """
    def __InferenceCallback(self, msg):
        # depends on what is send by the aruco publisher
        # the variables self.aruco_found and self.aruco_position
        pass



