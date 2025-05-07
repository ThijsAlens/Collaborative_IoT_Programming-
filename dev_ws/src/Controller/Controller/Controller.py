
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import time

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
        self.arucoInferenceController = ArucoInferenceController()
        self.clampController = ClampController()
        self.ultrasoundsController = UltrasoundInferenceController()

        self.current_state_index = 0
        self.states = [
            LookForBalls(self.get_logger(), self.ballInferenceController, self.wheelController),
            MoveToBalls(self.get_logger(), self.wheelController, self.ultrasoundsController),
            ClampBalls(self.get_logger(), self.clampController),
            LookForAruco(self.get_logger(), self.arucoInferenceController, self.wheelController),
            MoveToBalls(self.get_logger(), self.wheelController, self.ultrasoundsController),
            UnClampBalls(self.get_logger(), self.clampController)
        ]

        self.current_state = None

        time.sleep(2)

        self._set_state(self.states[self.current_state_index])

        self.create_timer(0.25, self.update)

    
    """
    Update method for the states, and to know when to transition states
    """
    def update(self):

        rclpy.spin_once(self.ballInferenceController, timeout_sec=0.25)
        #rclpy.spin_once(self.wheelController)
        rclpy.spin_once(self.arucoInferenceController, timeout_sec=0.25)
        #rclpy.spin_once(self.clampController)
        rclpy.spin_once(self.ultrasoundsController, timeout_sec=0.25)

        if(self.current_state.isFinished()):
            #if state is finished move on to next state
            self._find_next_state()
            
        else:
            #if not finished, update
            self.get_logger().info("Running update on state " + self.current_state.name)
            self.current_state.update()


    """
    Method to set current state
    """
    def _set_state(self, next_state : State):

        if(self.current_state is None):
            self.get_logger().info(f"Going to state {next_state.name}")
        else:
            self.current_state.stop()
            self.get_logger().info(f"Going from state {self.current_state.name} to state {next_state.name}")

        self.current_state = next_state
        self.current_state.start()

    """
    Find and set next state
    """
    def _find_next_state(self):
        self.current_state_index = (self.current_state_index + 1) % len(self.states)
        self._set_state(self.states[self.current_state_index])


    def destroy_nodes(self):
        #Since there are multiple nodes
        #Seperate method to make sure all are destroyed

        self.wheelController.destroy_node()
        self.ballInferenceController.destroy_node()

        self.destroy_node()
        
