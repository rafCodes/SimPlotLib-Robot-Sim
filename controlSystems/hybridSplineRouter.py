#returns list of point of class splinePathPoint
#the list it returns
import math

import RRT.rrt as rrt
import MainConfigClass
import inputProcessing
import waypointPathEditor
import routePlanner
import MainConfigClass
import SplineUtility
import SplineMotionPlanner
import toolbox
import numpy as np

import random

class lateralPathGenerator(object):

    def __init__(self):
        self.config = MainConfigClass.hybridSplineRouterConfig()
        self.splineMotionPlanner = SplineMotionPlanner.motionPlanningFullPath('') #'' means no splineBank file
        self.splineMotionPlanner.config.try_generation_limit = self.config.splineGenTries
        self.potentialFieldConfig = MainConfigClass.potentialFieldConfig()
        self.inputProcessingConfig = MainConfigClass.inputProcessingConfig()
        self.inputProcessor = inputProcessing.inputHandler()

    def generateAlternateRoute(self, searchLeft, searchRight, errorMarginObstacles, startKnots, endKnots, currentVelocity):
        """use spline gen to make the alternate route, return xList as 'Error' if there is a problem and the car needs
        to stop"""

        xList = []
        yList = []
        velocityList = []
        error = False

        tries = 0
        currentPivotPoint = 0
        pointXs, pointYs = self.generatePivotPoints(searchLeft, searchRight, errorMarginObstacles)
        pointXsSize = len(pointXs)

        while tries <= self.config.pivotTries:

            #check if need to regenerate pivot points
            if currentPivotPoint == pointXsSize:
                #need to regenerate points
                pointXs, pointYs = self.generatePivotPoints(searchLeft, searchRight, errorMarginObstacles)
                pointXsSize = len(pointXs)
                currentPivotPoint = 0

            goal = [endKnots[-1][0], endKnots[-1][1], self.inputProcessingConfig.tinyCubeWidth,
                    self.inputProcessingConfig.tinyCubeHeight]
            knots = startKnots + [pointXs[currentPivotPoint], pointYs[currentPivotPoint]] + endKnots

            #used to only start checking at current location
            startCheck = [startKnots][-1]

            splineInfo = self.splineMotionPlanner.returnSplineInfo(errorMarginObstacles, goal, currentVelocity, knots, startCheck)
            if splineInfo == []: #no spline found
                currentPivotPoint += 1
                tries += 1
            else: #spline found, all done
                xList = splineInfo.s_x
                yList = splineInfo.s_y
                velocityList = motionPlanner.returnVehiclePath(currentVelocity, splineInfo)
                return xList, yList, velocityList, error

        return xList, yList, velocityList, True

    def generatePivotPoints(self, searchLeft, searchRight, errorMarginObstacles):
        """generate the pivot points using the obstacle information and return as an array of X and an array of Y"""
        side = random.randint(0, 1)
        if side == 1: #right side
            window = searchRight
        else: #leftside
            window = searchLeft

        points = SplineUtility.sample_points_not_in_obs_array(errorMarginObstacles, window, self.config.pivotTries)
        pointsX = points[0]
        pointsY = points[1]
        return pointsX, pointsY

    def buildRoadNodeList(self, xList, yList):
        """build the path from x, and y lists in the RoadNode format"""
        roadNodeList = []
        currentIndex = 0
        maxIndex = len(xList)
        while currentIndex < maxIndex: #use zip?
            newNode = routePlanner.RoadNode(xList[currentIndex], yList[currentIndex])
            roadNodeList.append(newNode)
            currentIndex += self.config.indexJump

        return roadNodeList

    def hybridSplineMain(self, obstacles, targetObstacle, currentLocation, linkedListHead, linkedListTail, currentVelocity, heading, steeringAngle, method=None):
        """returns a list of road nodes and end node that signify the new path"""

        roadNodeHead = None
        roadNodeTail = None
        genError = False

        #need the first and last startKnot to be what is linked in waypointPathEditor

        #if no path is passed in
        if linkedListHead is None:
            return roadNodeHead, roadNodeTail, True

        startKnots = self.findFrontKnots(currentLocation, heading, steeringAngle)

        #validate start knots
        if len(startKnots) == 0:
            #no start knots
            return roadNodeHead, roadNodeTail, True

        endKnots = self.findBackKnots(obstacles, currentLocation, linkedListHead, linkedListTail)

        #validate end knots
        if len(endKnots) == 0:
            #no end knots
            return roadNodeHead, roadNodeTail, True

        genError = False
        if method == 'rrt':
            rrtGen = rrt.rrtPathGenerator()
            # TODO: rrt assumes currentLocation is (0, 0), but is that the case? Raphael says valid.
            # TODO: Convert end knot to a target region, or change rrt algo to allow point targets?
            # TODO: First convert last end knot to local coordinate frame using inputProcessing.py
            # TODO: Use tiny cube width (master class) to turn goal point into rectangle
            # TODO: searchWindow from rrtConfigClass should be increased to accomodate competition map
            searchWindow = rrtGen.config.SEARCH_WINDOW
            goalPoint = self.inputProcessor.convertGoalToLocalMap(endKnots[-1], heading)
            goalRectangle = [goalPoint[0], goalPoint[1], self.inputProcessingConfig.tinyCubeWidth,
                             self.inputProcessingConfig.tinyCubeHeight]
            output = rrtGen.generateRoute(searchWindow, obstacles, goalRectangle)
            if output is None:
                genError = True
            else:
                xList = output[0]
                yList = output[1]
                velocityList = np.zeros(len(xList))
        else:
            searchLeft, searchRight, errorMarginObstacles = self.findPivotBoundaries(obstacles, targetObstacle)

            # now, the pivot point is found here so that if there is a problem, it can keep trying until a certain
            # number
            xList, yList, velocityList, genError = self.generateAlternateRoute(searchLeft, searchRight,
                                                                               errorMarginObstacles, startKnots,
                                                                               endKnots, currentVelocity)

        #Check if there was a problem and a emergency stop is needed
        if genError:
            return roadNodeHead, roadNodeTail, genError
        else:
            if len(xList) == 0 or len(yList) == 0 or len(velocityList) == 0:
                return roadNodeHead, roadNodeTail, True
            else:
                roadNodeList = self.buildRoadNodeList(xList, yList, velocityList)

                roadNodeHead, roadNodeTail = waypointPathEditor.buildPath(roadNodeList)

        return roadNodeHead, roadNodeTail, genError

    def findFrontKnots(self, currentLocation, heading, steeringAngle):
        """return the knots that form the start of new segment. [[x, y], [x,y]...]"""

        startKnots = []
        startKnots.append(currentLocation)
        newKnots = self.generateFrontKnots(currentLocation, heading,
                                           steeringAngle, self.config.frontKnotsCount -1)
        startKnots.extend(newKnots)

        return startKnots[::-1] #need to flip order

    def generateFrontKnots(self, currentLocation, heading, steeringAngle, knotCount):
        """generate fake front knots for the spline generator"""

        steeringAngle = -1*steeringAngle+1.5708 #should convert so based off of [1,0]

        xSteering = math.cos(steeringAngle)
        ySteering = math.sin(steeringAngle)
        headingMag = np.linalg.norm(heading)

        xVector = (xSteering+heading[0]/headingMag)*self.potentialFieldConfig.car[3] #want height
        yVector = (ySteering+heading[1]/headingMag)*self.potentialFieldConfig.car[3] #want height

        xCurrent = currentLocation[0]
        yCurrent = currentLocation[1]

        newKnots = []
        while knotCount > 0:
            xCurrent -= xVector
            yCurrent -= yVector
            newKnots.append([xCurrent, yCurrent])
            knotCount -= 1

        return newKnots

    def findBackKnots(self, obstacles, linkedListHead, linkedListTail):
        """return the knots that form the end of the new segment [[x, y], [x,y]...]"""
        #current goal node is head of linked List

        backKnots = []
        currentNode = linkedListHead[:]

        invalidNodes = True

        #look at nodes for a valid node
        while invalidNodes and currentNode is not None:
            validNode = True
            obstacleIndex = 0
            while validNode and obstacleIndex < len(obstacles):
                #checks all obstacles for overlap, note, these are the regular sized obstacles
                distance = toolbox.getDistance(obstacles, currentNode.point, self.inputProcessingConfig.tinyCubeWidth, self.inputProcessingConfig.tinyCubeHeight)
                distance -= self.inputProcessingConfig.goalCircleRadius

                #checking if the knots are ok
                if distance <= 0:
                    validNode = False

                obstacleIndex += 1

            #checking if can now add to the current back knots
            if validNode:
                backKnots.append(currentNode.point)

                # checking if enough back knots found
                if len(backKnots) >= self.config.backKnotsCount:
                    invalidNodes = False

                #reset back knots if that knot is not safe
            else:
                backKnots = []

            #go to next node
            currentNode = currentNode.tail

        if backKnots < self.config.backKnotsCount:
            #means did not find a acceptable knot location, need to search some more like findFrontKnots
            newBackKnots = self.generateBackKnots(obstacles, linkedListHead, linkedListTail)
            backKnots.extend(newBackKnots)

        return backKnots

    def generateBackKnots(self, obstacles, linkedListHead, linkedListTail):
        """generate fake back knots since none were found"""
        #TODO
        newBackKnots = []

        return newBackKnots

    def findPivotBoundaries(self, obstacles, targetObstacle):
        """find the boundaries around the obstacle where a point can be placed"""
        #[x, y, w, h]
        errorMarginObstacles = []
        for obstacle in obstacles:
            tempObstacle = obstacle[:]
            tempObstacle[2] += self.config.pivotRadius*2
            tempObstacle[3] += self.config.pivotRadius*2
            errorMarginObstacles.append(tempObstacle)

        searchXBounds = targetObstacle[2]*self.config.targetObstacleWidthMultiplier + self.config.pivotRadius
        searchYBounds = targetObstacle[3]+ self.config.pivotRadius*2
        #is *2 since will be trying to navigate above and below obstacle potentially

        #making the two rectangles that points can be found in
        #x-w/2-searchXBounds/2, y, searchXBounds, searchYBounds
        searchLeft = [targetObstacle[0]-targetObstacle[2]/2-searchXBounds/2,targetObstacle[1], searchXBounds, searchYBounds]
        searchRight = [targetObstacle[0]+targetObstacle[2]/2+searchXBounds/2,targetObstacle[1], searchXBounds, searchYBounds]

        return searchLeft, searchRight, errorMarginObstacles


"""
some info for a ellipse around obstacle
       #reference https://stackoverflow.com/questions/433371/ellipse-bounding-a-rectangle

       x = cos(theta) * sqrt(2) * rect.width + x.center;
       y = sin(theta) * sqrt(2) * rect.height + y.center;

       x / A) ^ 2 + (y / B) ^ 2 = 1
       A / B = Rw / Rh

       Lets
       solve
       it: A = B * (Rw / Rh)
       (Rh / 2
       B) ^ 2 + (Rh / 2B) ^ 2 = 1
       Rh = sqrt(2) * B

       And
       final
       solution:
       A = Rw / sqrt(2)
       B = Rh / sqrt(2)

#need obstacles, current location, linked list, current velocity
detect that implant needed

determine start for implant - current location

safeEnd is found by finding a point that is outside the obstacle and far enough away

determine side knot(s) from the obstacle to use that does not hit another obstacle

knots for spline: side knot(s), safeEnd, safeEnd + 1, safeEnd +2... safeEnd(n-1) as needed
this ... will be dependent on velocity/tuning

safeEnd (n) is the goal for the spline

now we have the identity of the knots, start, and end as well as a list of points


with this, we will build a linkedlist of this path, with some of the points, start, and
safeEnd(n-1) as the tail

then this will be done and used by waypointPathEditor



CHANGE: only front of path will be edited, the end will be ignored and the last knot of the new
path will be the location of a new shortest path algorithm run start and then that will
be linked to linked list, getting rid of the of the 'ran out of path case'

If the goal is in the obstacle is still to be covered, perhaps a distance function from the obstacle
that will determine if the goal is in the obstacle and so it should be ignored/skipped?

when iterating searching for the knot, if the node has a none tag, means at end so then can mark that goal as complete,
otherwise proceed as normal

when getting middle knot, find angle between both knots, use equation for general radius of obstacle, find knot on 
circle around obstacle that is not is a angle +- angle of start or end knot, and pick random points for splines until one works



"""