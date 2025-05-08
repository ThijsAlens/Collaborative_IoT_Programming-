
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

        self.aruco_found: bool = False
        self.aruco_position: tuple[int, int]

        self.create_subscription(
            PositionStatus,
            "arucoDetection",
            self.__InferenceCallback,
            10
        )

        self.ballFound = False

        self.subscription = self.create_subscription(
            PositionStatus,
            "ballInference",
            self.__InferenceCallback,
            10
        )

        self.clampPublisher = self.create_publisher(
            String,
            'clamp',
            10
        )

        self.distance = 0
        self.speed = 0

        self.subscription = self.create_subscription(
            UltrasoundData, 
            "read_ultrasound_sensor",
            self.__InferenceCallback,
            10
        )

                
        self.wheels_publisher = self.create_publisher(
            String,
            'moveWheels',
            10
        )

        self.current_state_index = 0
        self.states = [
            LookForBalls(self.get_logger(), self),
            MoveToBalls(self.get_logger(), self),
            ClampBalls(self.get_logger(), self),
            LookForAruco(self.get_logger(), self),
            MoveToBalls(self.get_logger(), self),
            UnClampBalls(self.get_logger(), self)
        ]

        self.current_state = None

        time.sleep(5)

        self._set_state(self.states[self.current_state_index])

        self.create_timer(0.25, self.update)

    
    """
    Update method for the states, and to know when to transition states
    """
    def update(self):

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
        
    def __InferenceCallback(self, msg):
        self.aruco_found = msg.found
        self.position = (msg.x, msg.y)
        self.get_logger().info(f"ARUCO_controller heard:\tfound: {self.aruco_found}\t|\tposition: {self.position}")


    """
    Private method, do not call
    """
    def __InferenceCallback(self, msg):
        self.ballFound = msg.found


    """
    Open clamp (unsqueeze)
    """
    def open_clamp(self):
        self._publish_clamp_string("open")
        
    """
    Close clamp (squeeze)
    """
    def close_clamp(self):
        self._publish_clamp_string("close")

    """
    Private method, do not call
    """
    def _publish_clamp_string(self, string):
        msg = String()
        msg.data = string
        self.clampPublisher.publish(msg)

    """
    Private method, do not call
    """
    def __InferenceCallback(self, msg):

        self.distance = msg.distance
        self.speed = msg.speed

        #self.get_logger().info(f"RECIEVED: Distance: {self.distance}, speed: {self.speed}")

    """
    Move robot forward
    """
    def move_forward(self):
        self._publish_wheels_string("moveForward")

    """
    Move robot backwards
    """
    def move_backwards(self):
        self._publish_wheels_string("moveBackwards")

    
    """
    Rotate robot clockwise
    """
    def rotate_clockwise(self):
        self._publish_wheels_string("RotateCW")

    """
    Stop the robot
    """
    def stop(self):
        self._publish_wheels_string("stop")

    """
    Private method, do not call
    """
    def _publish_wheels_string(self, string):
        msg = String()
        msg.data = string
        self.wheels_publisher.publish(msg)



