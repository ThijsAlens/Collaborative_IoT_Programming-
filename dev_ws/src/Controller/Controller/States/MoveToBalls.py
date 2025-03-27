
from .State import *

"""
First state of robot
Robot should start rotating.
Once a ball is found the controller should move on to the next state
"""
class MoveToBalls(State):


    def __init__(self, logger, wheelController):
        super().__init__("Move to balls", logger)
        self.wheelController = wheelController

        self.logger.info("Moving forward")
        self.wheelController.move_forward()


    def update(self):
        pass

    def isFinished(self):
        #Something like distance sensor == small
        return False