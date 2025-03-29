
from .State import *

"""
First state of robot
Robot should start rotating.
Once a ball is found the controller should move on to the next state
"""
class LookForBalls(State):

    """
    Looking for balls needs the ball inference controller and the wheels controller
    params:
        ballInferenceController: Controller for finding balls
        wheelController: Controller for the wheels
    """
    def __init__(self, logger, ballInferenceController, wheelController):
        super().__init__("Looking for them balls", logger)
        self.ballInferenceController = ballInferenceController
        self.wheelController = wheelController

    """
    When the state starts, start rotating.
    """
    def start(self):
        self.logger.info("Rotating the wheels clockwise")
        self.wheelController.rotate_clockwise()


    """
    The move command is in start()
    Nothing to be done in update.
    """
    def update(self):
        """
        If the ball needs to be perfectly centered before moving on
        Here is where corrective measures would take place.
        Or in a different state "CentreBall" if we're feeling fancy
        """
        pass

    """
    When this state finished, make sure to stop rotating the robot.
    """
    def stop(self):
        self.logger.info("Stopping look for balls")
        self.wheelController.stop()


    def isFinished(self):
        #Once a ball is found, this state can terminate
        #A next state will move the robot to the ball
        return self.ballInferenceController.ballFound