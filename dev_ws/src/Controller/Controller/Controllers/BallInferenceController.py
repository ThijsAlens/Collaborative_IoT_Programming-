
import rclpy
from rclpy.node import Node
from custom_msg_aruco.msg import PositionStatus
"""
Controller for the ball inference
It will subscribe to topics regarding the Ball inference
They can call methods on this class, which will transmit to a topic.
"""
class BallInferenceController(Node):


    def __init__(self):

        super().__init__("BallInferenceController")
        
        self.ballFound = False

        self.subscription = self.create_subscription(
            PositionStatus,
            "ballInference",
            self.__InferenceCallback,
            10
        )


    """
    Private method, do not call
    """
    def __InferenceCallback(self, msg):
        self.ballFound = msg.found
        print("data from balls: "+ str(msg.found))
