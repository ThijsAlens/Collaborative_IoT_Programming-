
from .State import *

"""
First state of robot
Robot should start rotating.
Once a ball is found the controller should move on to the next state

This state is activated after clamping the ball
The robot should start looking for an aruco code
Once the code is found the controller should move on to the next state
"""
class LookForAruco(State):

    """
    Looking for aruco needs 2 controllers: the wheel controller and the aruco inference controller
    params:
        ballInferenceConArucoInferenceController: Controller for finding aruco codes
        wheelController: Controller for the wheels
    """
    def __init__(self, logger, arucoController, wheelController):
        super().__init__("Looking for the aruco code", logger)
        self.aruco_controller = arucoController
        self.wheel_controller = wheelController

    """
    When the state starts, start rotating.
    """
    def start(self):
        self.logger.info("Rotating the wheels clockwise")
        self.wheel_controller.rotate_clockwise()


    """
    The move command is in start()
    Nothing to be done in update.
    """
    def update(self):
        pass
        

    """
    When this state finished, make sure to stop rotating the robot.
    """
    def stop(self):
        self.logger.info("Stopping look for balls")
        self.wheel_controller.stop()


    def isFinished(self):
        #Once a aruco is found, this state can terminate
        #A next state will move the robot to the aruco
        return self.arucoController.aruco_found