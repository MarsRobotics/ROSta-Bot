__author__ = 'Matt Holland'

from state import *


def doNothing():
    return


class stateMachine:

    def __init__(self):
        self.states = []
        self.startState = self.addState("start", lambda: doNothing())
        self.currentState = self.startState

        return

    ##
    # addState
    #
    # Description: Adds a state to the state machine
    #
    # Parameters:
    #   name - The human readable name of the state
    #   onTick - lambda function to execute when this state is run
    #
    # Returns: The newly added state
    #
    def addState(self, name, onTick):
        newState = state(name, onTick)
        self.states.append(newState)

        return newState

    ##
    # getCurrentState
    #
    # Description: Gets the current state of the state machine.
    #
    # Returns: the current state of the state machine
    #
    def getCurrentState(self):
        return self.currentState

    ##
    # getStateByName
    #
    # Description: retrieves a state from this state machine by its human readable name
    #
    def getStateByName(self, name):
        for s in self.states:
            if s.name is name:
                return s
        return None

    ##
    # setCurrentState
    #
    # Description: Sets the current state of the state machine to the state specified.
    #   First however, it will verify that the state requirements are met before the transition
    #
    # TODO: Verify that the state requirements are met. (Robot is in the correct position for this state)
    # TODO: Check if the given state is actually in the state machine. Possibly overkill.
    def setCurrentState(self, newState):
        self.currentState = newState
        return True

    ##
    # tick
    #
    # Description: executes the states function and then transitions to the next function
    #
    def tick(self):
        self.currentState.onTick()
        self.currentState = self.currentState.getNextState()