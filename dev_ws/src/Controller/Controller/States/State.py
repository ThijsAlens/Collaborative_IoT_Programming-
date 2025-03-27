
"""
Represent a state in the statemachine
Should be abstract, but I don't know python
"""
class State():

    """
    When overriding add parameters for references to controllers
    params:
        name - name of the state
        logger - the logger from Node
    """
    def __init__(self, name, logger):
        self.name = name
        self.logger = logger

    """
    Controller has a timer which continuously calls this method.
    Override to have an update loop
    """
    def update(self):
        pass


    """
    When this method returns true, the controller will move on to the next state
    returns:
        bool - Whether the state is finished
    """
    def isFinished(self):
        pass