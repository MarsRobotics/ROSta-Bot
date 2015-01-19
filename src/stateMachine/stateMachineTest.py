__author__ = 'Matt Holland'

from stateMachine import *


def testFunction():
    return True

m = stateMachine()
s2 = m.addState("end", lambda: testFunction())
#TODO: why does this lambda function stuff work^v
m.startState.addTransition("toEnd", lambda: testFunction, s2)

print m.getStateByName("end").name


print m.getCurrentState().name
m.tick()
print m.getCurrentState().name
m.tick()
print m.getCurrentState().name
m.tick()
print m.getCurrentState().name




