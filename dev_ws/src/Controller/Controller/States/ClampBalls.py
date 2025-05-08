
from .State import *

"""
Once near the ball, close the clamp
"""
class ClampBalls(State):

    """
    Clamping balls only requires the clamp
    params:
        clampController
    """
    def __init__(self, logger, controller):
        super().__init__("Ready to close clamp", logger)
        self.clampController = controller

    """
    Close the clamp
    """
    def start(self):
        self.logger.info("starting state to close the clamp")
        self.clampController.close_clamp()

    def update(self):
        pass

    def stop(self):
        pass

    def isFinished(self):
        return self.clampController.closed