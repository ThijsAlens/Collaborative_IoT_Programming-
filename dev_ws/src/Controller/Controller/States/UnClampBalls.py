
from .State import *

"""
Once near the drop-off spot, open the clamp
"""
class UnClampBalls(State):

    """
    Unclamping balls only requires the clamp
    params:
        clampController
    """
    def __init__(self, logger, clampController):
        super().__init__("Ready to open clamp", logger)
        self.clampController = clampController

    """
    Open the clamp
    """
    def start(self):
        self.logger.info("starting state to open the clamp")
        self.clampController.open_clamp()

    def update(self):
        pass

    def stop(self):
        pass

    def isFinished(self):
        return self.clampController.opened