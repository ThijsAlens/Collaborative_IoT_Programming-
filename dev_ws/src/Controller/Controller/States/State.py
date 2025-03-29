
"""
Represent a state in the statemachine
Should be abstract, but I don't know python
"""
class State():

    """
    When overriding add parameters for references to controllers.
    This is for configuring the state, not starting the robot
    params:
        name: name of the state
        logger: the logger from Node
    """
    def __init__(self, name, logger):
        self.name = name
        self.logger = logger

    """
    start is called when the state should commence. 
    Here first orders should be given.
    """
    def start(self):
        pass

    """
    Controller has a timer which continuously calls this method.
    Override to have an update loop
    """
    def update(self):
        pass


    """
    stop is called when transitioning away from the state.
    The controllers should be reset to return to a default state.
    """
    def stop(self):
        pass


    """
    When this method returns true, the controller will move on to the next state
    returns:
        bool - Whether the state is finished
    """
    def isFinished(self):
        pass