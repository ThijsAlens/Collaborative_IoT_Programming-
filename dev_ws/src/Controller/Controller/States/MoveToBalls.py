
from .State import *

"""
Once a ball is found, and centered move forward to intercept the ball.
"""
class MoveToBalls(State):

    """
    Moving to balls needs the wheels, and sensor for distance
    params:
        wheelController
        # DistanceSensorController
    """
    def __init__(self, logger, wheelController, ultrasoundsController):
        super().__init__("Move to balls", logger)
        self.wheelController = wheelController
        self.ultrasoundsController = ultrasoundsController

    """
    start moving forward
    """
    def start(self):
        self.logger.info("starting state move to balls")
        self.wheelController.move_forward()

    def update(self):
        pass

    """
    When the state stops, make sure to stop the wheels
    """
    def stop(self):
        self.wheelController.stop()

    def isFinished(self):
        return self.ultrasoundsController.distance < 3