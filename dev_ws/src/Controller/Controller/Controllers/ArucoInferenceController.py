
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
        self.aruco_position: tuple[int, int]

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
        self.get_logger().info(f"ARUCO_controller heard:\tfound: {self.aruco_found}\t|\tposition: {self.position}")



