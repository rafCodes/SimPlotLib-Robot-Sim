#!/usr/bin/env python
import math
import numpy as np


import MainConfigClass
import toolbox


class inputHandler(object):

    def __init__(self):
        self.config = MainConfigClass.inputProcessingConfig()

    def getGoal(self, vectorToGoal):
        # def getGoal(self, vectorToGoal : Array):
        """return the goal that is on the edge of or inside the circle that surrounds the car"""
        vectorToGoalMagnitude = toolbox.fastNpLinAlg(vectorToGoal)
        if vectorToGoalMagnitude > self.config.goalCircleRadius:
            factor = self.config.goalCircleRadius / vectorToGoalMagnitude
            vectorToGoal[0] *= factor #done in this order to cast to float
            vectorToGoal[1] *= factor

        return vectorToGoal

    def convertGoalToLocalMap(self, goalVector, heading):
        # def convertGoalToLocalMap(self, goalVector: Array, heading: List):
        """return the goalVector in relation to the local map"""

        headingAngleRadians = math.acos(np.dot(heading, [0, 1]) / (toolbox.fastNpLinAlg(heading))) #removed abs
        headingSide = toolbox.getSide([0, 1], heading)

        # https://stackoverflow.com/questions/14607640/rotating-a-vector-in-3d-space
        # https://mathworld.wolfram.com/RotationMatrix.html
        # https://matthew-brett.github.io/teaching/rotation_2d.html

        c = math.cos(headingAngleRadians)  # cos
        s = math.sin(headingAngleRadians)  # sin

        if headingSide == 1:  # right
            # need to rotate goal counter-clockwise
            x = goalVector[0] * c - goalVector[1] * s
            y = goalVector[0] * s + goalVector[1] * c

        else:  # left
            # need to rotate goal clockwise
            x = goalVector[0] * c + goalVector[1] * s
            y = -goalVector[0] * s + goalVector[1] * c

        return [x, y]

    def convertLIDARToObstacles(self):
        """shift lidar points and turn them into obstacles so that their distances can be accurately measured"""

        newPoints = []
        for point in self.cloudPoints:
            x = point[0] - self.config.LIDARShift[0]
            y = point[1] - self.config.LIDARShift[1]
            point = [x, y, self.config.tinyCubeWidth, self.config.tinyCubeHeight]
            newPoints.append(point)
        return newPoints

    def getClosestLidarPointHelper(self, lidarPoints):
        # def getClosestLidarPoint(self, lidarPoints : List):
        """return the closest lidar point(s) [x,y] and distance from the x,y,z lidar points"""
        # note, only the last closest point will be returned

        if lidarPoints == []:
            return [], 999999

        lidarDistances = []

        for point in lidarPoints:
            lidarDistances.append(toolbox.fastNpLinAlg(point[0:2]))

        # now find smallest distance
        minDistance = 9999999999
        minLidarPoint = 0
        count = 0

        # two sets made here to reduce computations when possible
        if self.config.LIDARShift == [0, 0]:
            for point in lidarPoints:
                distance = toolbox.fastNpLinAlg(point[0:2])
                if distance < minDistance:
                    minLidarPoint = count
                    minDistance = distance
                count += 1
        else:
            for point in lidarPoints:
                point = [point[0] + self.config.LIDARShift,
                         point[1] + self.config.LIDARShift]  # this is added if needed
                distance = toolbox.fastNpLinAlg(point)  # this line was also changed
                if distance < minDistance:
                    minLidarPoint = count
                    minDistance = distance
                count += 1

        return lidarPoints[minLidarPoint][0:2], minDistance

    def findGoalDirection(self, currentGoal, currentLocation):
        # def findGoalDirection(self, currentGoal : List, currentLocation : List):
        """returns the direction of the goal from a point"""
        return [currentGoal[0] - currentLocation[0], currentGoal[1] - currentLocation[1]]
