import numpy as np

import toolbox

def imminentCollision(distanceXY, currentVelocity):
    """P controller for the tollerance of an obstacle infront of the vehicle, if too close returns True"""
    return False
    # #checking if hitting an obstacle in front of it and if can turn past its width at current velocity (last part not tested because first test means
    # #that the car can make a turn to be parallel to the obstacle
    #
    # #determine where obstacle is to see if matters:
    # distanceXY = distanceXY / np.linalg.norm(distanceXY)
    # calculatedAngle = math.acos(np.dot(np.linalg.norm(distanceXY), np.array([0, 1])))
    # #TODO make it so do not need to calculate every time
    #
    # # now need to determine which side the angle is on
    # calculatedAngle *= toolbox.getSide([0, 1], distanceXY)
    #
    # safe = computeDanger(calculatedAngle, currentVelocity)
    #
    # return safe


def computeDanger(calculatedAngle, currentVelocity):
    """This defines a two variable mathamatical function that is dependent on the car's
    physical attributes such as turning rate, size, inertia and must be defined at a later date,
    for now, always returns that NOT going to crash"""

    return True

import ConfigLoader

def obstacleDirectlyInfront(intialVelocity : float, obstacleMap):
    return False