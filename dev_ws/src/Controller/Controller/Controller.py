
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from .Controllers import *
from .States import *

def main(args = None):

    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    controller.destroy_nodes()
    rclpy.shutdown()


class Controller(Node):

    def __init__(self):
        
        super().__init__("Controller")

        #Create controllers, which will internally subscribe to topics
        self.wheelController = WheelController()
        self.ballInferenceController = BallInferenceController()

        self.current_state = None
        self.set_state(LookForBalls(self.get_logger(), self.ballInferenceController, self.wheelController))

        self.create_timer(0.5, self.update)

    
    """
    Update method for the states, and to know when to transition states
    """
    def update(self):

        if(not self.current_state.isFinished()):
            #If the state is not finished, update it
            self.get_logger().info("Running update on state " + self.current_state.name)
            self.current_state.update()
        else:
            #else we can move on to the new state
            pass

    """
    Method to set current state
    """
    def set_state(self, next_state : State):

        if(self.current_state is None):
            self.get_logger().info(f"Going to state {next_state.name}")
        else:
            self.current_state.stop()
            self.get_logger().info(f"Going from state {self.current_state.name} to state {next_state.name}")

        self.current_state = next_state
        self.current_state.start()

    def destroy_nodes(self):
        #Since there are multiple nodes
        #Seperate method to make sure all are destroyed

        self.wheelController.destroy_node()
        self.ballInferenceController.destroy_node()

        self.destroy_node()
        
