#!/usr/bin/env python

import toolbox
from controlSystems import inputProcessing
import MainConfigClass

# https://www.youtube.com/watch?v=MQjeqvbzhGQ&list=PL_onPhFCkVQhuPiUxUW2lFHB39QsavEEA&index=64
# https://www.youtube.com/watch?v=j6uQMgajwcU&list=PL_onPhFCkVQhuPiUxUW2lFHB39QsavEEA&index=65
# https://www.youtube.com/watch?v=11gOb-ehmoo&amp=&index=66

class potentialFieldMapNode(object):
    """builds a potential field based off of obstacles and paths given by vision"""

    def __init__(self):
        """initialize class with config and ROS connections needed"""

        self.config = MainConfigClass.potentialFieldConfig()
        self.inputHandler = inputProcessing.inputHandler()
        self.LOGFLAG = self.config.LOGFLAG
        self.currentGoal = []
        self.goal = [0, 0]
        self.currentLocation = []
        self.cloudPoints = []
        #TODO change the ROS connection so no obstacles are passed
        self.obstacles = [] #MasterClass.OBSTACLES
        self.currentHeading = self.config.initialHeading
        self.currentVelocity = 0
        self.rayVector = []

        self.currentGear = MainConfigClass.FORWARD

        self.magFactor = 1
        self.collision = False

    def getClosestObstacle(self, obstacles, point):
    # def getClosestObstacle(self, obstacles : List, point : Array, config):
        """given a list of obstacles and a point, return the distance to the closest obstacle to the point and the closest obstacle"""
        minimumDistance = 999999999
        chosenObstacle = None
        XYLengths = None

        for obs in obstacles:
            coordinates = toolbox.getDistance(obs, point, self.config.car[2], self.config.car[3])
            distance = toolbox.fastNpLinAlg(coordinates)

            if distance < minimumDistance:
                minimumDistance = distance
                XYLengths = coordinates
                chosenObstacle = obs

        return minimumDistance, chosenObstacle, XYLengths

    def getPotentialAttracting(self, goal, point): #fa, used to create function to get robot to the goal
    # def getPotentialAttracting(self, goal : Array, point : Array, config): #fa, used to create function to get robot to the goal
        """given a goal point and a point, both numpy arrays, find the attracting POTENTIAL of this distance"""

        #attractingFactor is a config value
        vector = [point[0]-goal[0], point[1]-goal[1]]
        return (vector[0]*vector[0]+vector[1]*vector[1])*self.config.attractingFactor*-1

    def getPotentialRepelling(self, obsOrLIDARData, point):
    # def getPotentialRepelling(self, lidarPoints: List, point: Array, config):
        """given a list of 2D object, and a point, return the repelling POTENTIAL between the point and the closest object"""

        #if self.config.vision:
        distance, obstacle, distanceXY = self.getClosestObstacle(obsOrLIDARData, point) #obstacles : List replaces lidarPoints : List
        #else:
        #    distanceXY, distance = self.getClosestLidarPointHelper(obsOrLIDARData) #will not log information

        # check if obstacle is too far away to matter, relavantDistance is a config value
        if distance > self.config.relevantDistance:
            return 0

        if distance <= 0:
            self.collision = True
            return self.config.maxRepelling

        # repellingFactor is a config value, self.currentVelocity*self.config.maxSpeedFactor used to have more or less repelling depending on speed
        # removed * self.currentVelocity*self.config.maxSpeedFactor
        factor = (1.0 / distance) - (1.0 / self.config.repellingFactor)
        repelling = self.config.repellingScalingParameter * factor * factor

        if repelling > self.config.maxRepelling:
            repelling = self.config.maxRepelling

        return repelling

    def pointPotential(self, obsOrLIDARData, goal, point):
    # def pointPotential(self, lidarPoints: List, goal: Array, point: Array):
        """return the total potential at a point"""

        repelling = self.getPotentialRepelling(obsOrLIDARData, point)  # replace lidarPoints with obstacles
        attracting = self.getPotentialAttracting(goal, point)

        pointPotential = repelling + attracting
        if pointPotential < self.config.maxPotential:
            pointPotential = self.config.maxPotential

        return pointPotential

    def getPotentialAttractingGradient(self, goal, point):  # fA, used to create function to get robot to the goal
        # def getPotentialAttractingGradient(self, goal: Array, point: Array, config):
        """given a goal point and a point, both numpy arrays, find the attracting POTENTIAL gradient at the point, vector pointing away from goal"""

        # attractingFactor is a config value

        # goalMag = toolbox.fastNpLinAlg(goal)
        # if goalMag < MasterClass.WAYPOINT_DETECTION_DISTANCE:
        #     goal[0] *= MasterClass.WAYPOINT_DETECTION_DISTANCE/goalMag
        #     goal[1] *= MasterClass.WAYPOINT_DETECTION_DISTANCE/goalMag

        xValue = self.config.attractingFactor * 2 * (goal[0] - point[0] - self.config.car[0]) * -1
        yValue = self.config.attractingFactor * 2 * (goal[1] - point[1] - self.config.car[1]) * -1
        return [xValue, yValue]

    def getPotentialRepellingGradient(self, obsOrLIDARData, point):
    # def getPotentialRepellingGradient(self, lidarPoints: List, point: Array, config):
        """given a list of 2D object, and a point, return the repelling POTENTIAL gradient between the point and the closest object"""

        #if self.config.vision:
        distance, chosenObstacle, XYLengths = self.getClosestObstacle(obsOrLIDARData, point) #obstacles : List replaces lidarPoints : List

        #else:
        #    XYLengths, distance = self.getClosestLidarPointHelper(obsOrLIDARData) #will not log information
        #    if XYLengths == []:
        #        return np.array([0, 0])
            #closestPoint = self.convertGoalToLocalMap(XYLengths, heading) debug code

        # check if obstacle is too far away to matter or is too close, relavantDistance is a config value
        if distance > self.config.relevantDistance:
            return [0, 0]

        if distance == 0:
            # on edge of obstacle
            self.collision = True
            return [0, 0]

        # now finding x, y components of vector
        xComponent = XYLengths[0]
        yComponent = XYLengths[1]

        # repellingFactor is a config value

        #https://www.wolframalpha.com/input/?i=derivative+of+%281%2F%28xx%2Byy%29**.5-1%2Fb%29**2
        #*self.currentVelocity*self.config.maxSpeedFactor* makes it so faster you are going, more repelling from scaling parameter
        # removed *self.currentVelocity*self.config.maxSpeedFactor
        xValue = xComponent*(2.0*self.config.repellingScalingParameter*(distance-self.config.repellingFactor))/(self.config.repellingFactor*distance*distance*distance*distance)
        yValue = yComponent*(2.0*self.config.repellingScalingParameter*(distance-self.config.repellingFactor))/(self.config.repellingFactor*distance*distance*distance*distance)

        return [xValue, yValue]

    def pointPotentialGradient(self, obsOrLIDARData, goal, point, rayGiven = False):
    # def pointPotentialGradient(self, lidarPoints: List, goal: Array, point: Array):
        """return the total GRADIENT of the potential at a point"""

        if rayGiven:
            #for passing ray vector, convert to a tiny obstacle
            #might need to shift from camera location
            x = obsOrLIDARData[0] - self.config.cameraShift[0]
            y = obsOrLIDARData[1] - self.config.cameraShift[0]
            obsOrLIDARData = [[x, y, self.inputHandler.config.tinyCubeWidth, self.inputHandler.config.tinyCubeHeight]]

        attracting = self.getPotentialAttractingGradient(goal, point)

        if self.magFactor != 1:
            #need to edit magnitude of attracting
            attractingMag = toolbox.fastNpLinAlg(attracting)
            attracting[0] /= attractingMag
            attracting[0] *= self.magFactor

            attracting[1] /= attractingMag
            attracting[1] *= self.magFactor

        if obsOrLIDARData == []:
            #no obstacles so no repelling needed
            pointPotentialGradient = attracting
            pointPotentialGradient[0] *= -1
            pointPotentialGradient[1] *= -1
        else:
            repelling = self.getPotentialRepellingGradient(obsOrLIDARData, point)  # replace lidarPoints with obstacles
            if self.collision:
                self.collision = False
                return [0, 0]
            pointPotentialGradient = ([-1*(repelling[0] + attracting[0]), -1*(repelling[1] + attracting[1])])

        pointPotentialGradientMagnitude = toolbox.fastNpLinAlg(pointPotentialGradient)
        magnitudeFraction = self.config.maxPotentialAbs / float(pointPotentialGradientMagnitude)

        if magnitudeFraction < 1:
            # need to divide vector so that it is the same magnitude as maxPotential
            pointPotentialGradient[0] *= magnitudeFraction
            pointPotentialGradient[1] *= magnitudeFraction
        return pointPotentialGradient

    def directionUpdateHelper(self, rayVector = []):
        """return the direction needed to follow from obstacle or LIDAR data, goal, and point information"""

        if self.currentGoal == []:
            # means car needs to be stopped, all goals have been found, publish a 0,0 vector
            return []

        else:

            # no longer used by left in in case want to see example or use at a later point
            if len(self.currentGoal) == 3 and self.currentGoal[2] != MainConfigClass.IGNORE_VELOCITY:
                # means a different velocity has been passed
                self.magFactor = self.currentGoal[2]
            else:
                self.magFactor = 1

            if self.currentGear == MainConfigClass.FORWARD:
                self.config.attractingFactor = self.config.attractingForwardFactor
            else:
                self.config.attractingFactor = self.config.attractingReverseFactor

            # find the goal and put it on the local map
            #these three now from input processing
            vectorToGoal = self.inputHandler.findGoalDirection(self.currentGoal[0:2], self.currentLocation)
            convertedGoal = self.inputHandler.convertGoalToLocalMap(vectorToGoal, self.currentHeading)
            self.goal = self.inputHandler.getGoal(convertedGoal)

            point = self.config.car[0:2]

            if self.config.vision:
                if self.config.rayVector and rayVector != []: ## add config for rayVector on off and not empty list
                    self.rayVector = rayVector

                    potentialGradient = self.pointPotentialGradient(rayVector, self.goal, point, True)
                else:
                    potentialGradient = self.pointPotentialGradient(self.obstacles, self.goal, point)
            else:
                LIDARObstacles = self.inputHandler.convertLIDARToObstacles()
                potentialGradient = self.pointPotentialGradient(LIDARObstacles, self.goal, point)

            potentialGradient = [potentialGradient[0], potentialGradient[1]]

            return potentialGradient