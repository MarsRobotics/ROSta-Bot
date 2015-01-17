__author__ = 'Matt Holland'


class transition:

    nextTransitionId = 0

    ##
    # transition
    #
    # Description: Creates
    #
    # Parameters:
    #   name - human readable name of the transition
    #   canTransition - A lambda function to check if the conditions for this transition are true
    #   nextState - the state object to transition to
    #
    def __init__(self, name, canTransition, nextState):

        self.canTransition = canTransition
        self.nextState = nextState
        self.name = name
        self.id = ++transition.nextTransitionId

        return