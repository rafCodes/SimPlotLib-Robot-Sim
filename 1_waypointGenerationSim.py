#!/usr/bin/env python
from controlSystems import vehicleControl
from controlSystems import waypointManager
import MainConfigClass

import simulationBaseClass
import SimMainConfigClass

from controlSystems.routePlanner import *
import toolbox

import numpy as np
import xlrd
import math
import time

class waypointGen(simulationBaseClass.simulationInterface):

    def __init__(self, simulationConfig, displayConfig, path, maps = '', img = '', carData = ''):

        super(waypointGen, self).__init__(simulationConfig, displayConfig, maps, path)

        self.waypointCount = len(self.map.waypoints)
        if 0 in self.map.waypoints:
            if self.waypointCount > 1:
                self.waypointCount -= 1 #this avoids an extra index when loading a map

        self.vC = vehicleControl.vehicleControlNode()
        self.potentialFieldConfig = MainConfigClass.potentialFieldConfig()

        self.wM = waypointManager.goalOverseerNode(self.map.waypoints, self.map.edges, MainConfigClass.GOALS)

        self.simPathList = []
        self.simPathIndex = 0
        self.simPathGoalPaths = {}
        self.simPathLength = 0

        self.mapStart = True # so that resizing does not get upset when map opens with 0 width and height since just initalized

        #for driving car
        self.headings = []
        self.locations = []
        self.timeStamps = []
        self.velocities = []

        self.collisionTimeStamps = []

        if carData != '':
            #Excel time
            #https://www.geeksforgeeks.org/reading-excel-file-using-python/
            self.convertCarData(carData)

        self.img = img

        self.obsPair = False
        self.waypointTypeStart = False
        self.waypointNotObs = True
        self.waypointOrOBS = {True: 'Waypoints and Edges', False: 'Obstacle'}
        self.clickCache = {'mapLoc': []} #[x, y, waypoint number]

        self.car = False  #used to show car animation
        self.carLocation = []
        self.carHeading = 0
        self.showCarPath = False

        #RRT stuff
        self.rrtData = []
        self.rrtOn = False

        print('INSTRUCTIONS')
        print('Left click place point or edges by clicking near two points')
        print('2 clicks on a waypoint remove it')
        print('Right click removes edges by clicking near two points')
        print('Right clicking on a obstacle removes it')

    def setup(self):
        """prepare the simulation environment"""
        print("Environment setup complete")

    def populateGUI(self):
        """update the GUIs from the config, can also be used to update the initial value during a run"""

        self.populateBaseGUI()

        #switch between making obstacles or making waypoints
        self.config.buttons['makeObstacleORMakeWaypoint']['function'] = self.makeObstacleORMakeWaypoint

        #place start location
        self.config.buttons['placeStart']['function'] = self.placeStart

        # add car path functionality
        self.config.buttons['carPath']['function'] = self.toggleCarPath

        # add drive car functionality
        self.config.buttons['driveCar']['function'] = self.driveCar

        # add button to simulate a path
        self.config.buttons['simPath']['function'] = self.simPath

        # add button to go forward in a path
        self.config.buttons['simPathIncrement']['function'] = self.simIncrement

        # add button to go backward in a path
        self.config.buttons['simPathDecrement']['function'] = self.simDecrement

        # add slider for time
        self.config.sliders['driveCarTimeChange']['function'] = self.changeDriveCarTime
        self.config.sliders['driveCarTimeChange']['initialValue'] = self.config.timeIndexIncrement

        # add to plot
        self.display.loadGUI(self.config.buttons, self.config.sliders, self.config.textBoxes)

    def populateGraph(self):
        """set up objects - rectangles, circles, and arrows - and then populate the graph with those objects"""

        rectangles, circles, arrows = self.populateBaseGraph()

        if self.waypointNotObs == False:
            #plot temp points
            if self.obsPair == True:
                circles.append([[self.clickCache['mapLoc'][0]], self.config.waypointRadius, ['blue', True]])

        if self.showCarPath:
            i = 0
            carPoints = []
            while i < len(self.locations):
                carPoints.append([self.locations[i][0] + self.config.shiftX, self.locations[i][1] + self.config.shiftY])
                i += self.config.timeIndexIncrement

            circles.append([carPoints, self.config.carPathRadius, ['red', True]])

        self.display.plotShapes(rectangles, circles, arrows)

    def simPath(self, event):
        """simulate a path with waypointManager"""
        self.simPathList = []
        self.simPathIndex = 0
        self.simPathGoalPaths = {}
        self.simPathLength = 0

        count = 1

        regenerateHeading = True
        lastWasNone = False

        if self.wM.config.breadcrumbMode:
            toolbox.warning('Breadcrumb mode is active in Waypoint Manager')

        while self.wM.currentGoal != [] and self.wM.currentPath is not None and self.wM.targetGoal != []:
            #makes heading for next path
            if regenerateHeading:
                self.regenerateHeading(count)
                count += 1
                regenerateHeading = False

            self.simPathList.append(self.wM.currentGoal)
            self.wM.location = self.wM.currentGoal
            self.wM.findNextWaypoint_mainHelper()

            #logic so that the heading is generated from the next fresh path
            if lastWasNone:
                lastWasNone = False
                regenerateHeading = True
            if self.wM.currentPath.tail == None:
                lastWasNone = True

        if self.simPathList != []:
            self.simPathIndex = 0
            self.circleData.append([self.simPathList[:self.simPathIndex], self.config.waypointPathSimRadius, ['red', False]])

        if self.wM.currentPath is None:
            print('No path available, no path simulated')
        else:
            print('Path has been simulated')

        self.buttonUpdate()

    def regenerateHeading(self, count):
        """regenerate the heading for the next place for the sim to path to"""
        outputNode = self.wM.currentPath
        path = []
        #make list of nodes to travel on
        while outputNode is not None:
            path.append([outputNode.point.x, outputNode.point.y])
            outputNode = outputNode.tail
        self.simPathGoalPaths[count] = path

        # find vector between last two nodes that points towards goal for final vector/heading
        vectorStart = [self.wM.currentPathTail.head.point.x, self.wM.currentPathTail.head.point.y]
        vectorEnd = [self.wM.currentPathTail.point.x, self.wM.currentPathTail.point.y]
        angle, vector = toolbox.getVectorAngle(vectorStart, vectorEnd)

        #add vector to end of simPathGoalPaths
        self.simPathGoalPaths[count].append([vector, angle])
        self.wM.heading_angle = angle

    def simIncrement(self, event):
        """go to the next part of the path"""
        self.simPathIndex += 1

        if self.simPathList != []:
            if self.simPathIndex > len(self.simPathList)-1:
                self.simPathIndex -= 1
                print('At end of simulated path')
                self.buttonUpdate()
            self.circleData.append([self.simPathList[:self.simPathIndex+1], self.config.waypointPathSimRadius, ['red', False]])
            self.simPathLength = toolbox.pathDistance(self.simPathList[:self.simPathIndex+1])
            self.simEdges()
            print('Current path length: {}'.format(self.simPathLength))
            self.buttonUpdate()
        else:
            print('Need to simulate path')

    def simDecrement(self, event):
        """go back one part of the path"""
        if self.simPathList != []:
            self.simPathIndex -= 1
            if self.simPathList != []:
                if self.simPathIndex < 0:
                    self.simPathIndex += 1
                    print('At the start of the path')
                self.circleData.append([self.simPathList[:self.simPathIndex+1], self.config.waypointPathSimRadius, ['red', False]])
                self.simPathLength = toolbox.pathDistance(self.simPathList[:self.simPathIndex + 1])

                self.simEdges()
            else:
                print('Need to start the path simulation')
            print('Current path length: {}'.format(self.simPathLength))
            self.buttonUpdate()
        else:
            print('Need to simulate path')

    def simEdges(self):
        """plot the edges for the simulated path"""
        if self.simPathList == []:
            print('Need to start the path simulation')
        else:
            for i in range(1, self.simPathIndex+1):
                pointOne = self.simPathList[i-1]
                pointTwo = self.simPathList[i]
                self.arrowData.append([pointOne[0], pointOne[1], pointTwo[0]-pointOne[0], pointTwo[1]-pointOne[1], self.config.arrowWidth, self.config.arrowHeadSize])

    def convertCarData(self, carData):
        """convert the Excel file of the car data so it can be shown"""
        # location (x, y), heading vector (x, y), heading angle in radians, velocity, seconds, milliseconds
        wb = xlrd.open_workbook(carData)
        sheet = wb.sheet_by_index(0)

        for i in range(sheet.nrows):
            self.locations.append([(sheet.cell_value(i, 0)), (sheet.cell_value(i, 1))])

        sheet.cell_value(0, 1)
        for i in range(sheet.nrows):
            self.headings.append([(sheet.cell_value(i, 3)), (sheet.cell_value(i, 4))])

        sheet.cell_value(0, 4)
        for i in range(sheet.nrows):
            self.timeStamps.append((sheet.cell_value(i, 8)) + (sheet.cell_value(i, 9)) / 1000)

        for i in range(sheet.nrows):
            self.velocities.append(sheet.cell_value(i, 7))

    def randomDebug(self, event):
        """place to put random functions to test and debug without having to make a new button for it immediately"""
        # print(self.clickCache)
        while self.simPathIndex < len(self.simPathList)-1:
            self.simIncrement(1)
            time.sleep(.2)
        # for path in self.simPathGoalPaths:
        #     print('Heading:', self.simPathGoalPaths[path][-1], 'Path:', self.simPathGoalPaths[path][:-1])

    def changeDriveCarTime(self, val):
        """increase time increment by 5"""
        self.config.timeIndexIncrement = int(self.display.sliders[self.config.sliders['driveCarTimeChange']['name']].val)
        self.buttonUpdate()

    def toggleCarPath(self, event):
        """toggle show car path"""
        self.showCarPath = not self.showCarPath

        self.buttonUpdate()

    def driveCar(self, event):
        """run the animation of the car driving"""

        print('Drive Started')

        self.car = not self.car
        if self.car:
            self.collisionTimeStamps = []

        #set up dimentions
        self.carLocation = self.potentialFieldConfig.car

        lenTime = len(self.timeStamps)
        i = 0
        while i < lenTime and self.car:

            #draw the next car frame

            #print('Location: ' , self.locations[i])
            self.carLocation[0] = self.locations[i][0] + self.config.shiftX
            self.carLocation[1] = self.locations[i][1] + self.config.shiftY

            self.vC.currentVelocity = self.velocities[i]
            self.vC.currentLocation = [self.carLocation[0], self.carLocation[1]]

            #self.arrowData.append([self.carLocation[0], self.carLocation[1], 50 * self.headings[i][0], 50 * self.headings[i][1], 2, 5])

            if i > 20:
                angle, vector = toolbox.getVectorAngle([self.locations[i-20][0] + self.config.shiftX, self.locations[i-20][1] + self.config.shiftX], [self.carLocation[0], self.carLocation[1]])
                self.arrowData.append([self.carLocation[0], self.carLocation[1], 40* vector[0], 40*vector[1], 2, 5])
                self.arrowData.append([self.carLocation[0], self.carLocation[1], math.cos(angle)*20, 20*math.sin(angle), 2, 5])


            heading = np.array(self.headings[i])

            headingAngleRadians = np.fabs(math.acos(np.dot(heading, [0, 1]) / (np.linalg.norm(heading))))
            headingSide = toolbox.getSide([0, 1], heading)

            self.carHeading = headingAngleRadians*headingSide #TODO this work? I think so, I also did this in Vehicle Controls

            if self.config.showTime:
                print(self.timeStamps[i])
            checkPos = self.vC.checkPosition(self.timeStamps[i])
            if checkPos:
                self.collisionTimeStamps.append(self.timeStamps[i])

            self.buttonUpdate()

            #then pause for simWaitTime seconds
            time.sleep(self.config.simWaitTime)

            i += self.config.timeIndexIncrement

        self.car = False
        print('Drive Complete')

    def makeObstacleORMakeWaypoint(self, event):  #############
        """button to swap between making waypoints and edges and making obstacles,
        swaps text depending on mode in corner"""
        self.waypointNotObs = not self.waypointNotObs
        self.clickCache['mapLoc'] = []
        print('Points create', self.waypointOrOBS[self.waypointNotObs])
        #self.config.textBoxes['obsORWP']['initialText'] = self.waypointOrOBS[self.waypointNotObs]

    def placeStart(self, value):
        """when clicked, places next waypoint as the start waypoint,
        overriding previous waypoints if needed"""
        self.waypointTypeStart = True

    def addWayPoint(self, x, y):
        """adds a waypoint to the map"""
        while self.waypointCount in self.map.waypoints:
            self.waypointCount += 1
        self.map.waypoints[self.waypointCount] = [x, y]

    def addStartWaypoint(self, x, y):
        self.map.waypoints[0] = [x, y]
        self.waypointTypeStart = False

    def editEdge(self, button):
        """adds or removes an edge to the map"""
        newEdge = [self.clickCache['mapLoc'][0][2], self.clickCache['mapLoc'][1][2]]
        inEdge = self.checkIfEdge(newEdge)
        if inEdge == 0:  # not in edges
            if button == 1:  # add the edge if not already in edges
                self.map.edges.append(newEdge)
            self.clickCache['mapLoc'] = []
        else:
            if button == 1:
                # already in cache so no reason to add
                self.clickCache['mapLoc'] = []
            else:
            # remove the edge
                if inEdge == -1:
                    # flip the newEdge
                    newEdge = newEdge[::-1]
                self.map.edges.remove(newEdge)
                self.clickCache['mapLoc'] = []

    def editObstacles(self, x, y, button):
        """adds an obstacle to the map"""
        inOb, Ob = self.inObs(x, y)
        if inOb and button == 3:
            # inside obstacle, need to remove obstacle
            self.map.obstacles.remove(Ob)
        elif self.obsPair == True:
            # already have one point, need to make obstacle
            self.clickCache['mapLoc'].append([x, y])
            newObs = self.makeObFromCorner()
            if newObs != []:
                self.map.obstacles.append(newObs)
            self.obsPair = False
            self.clickCache['mapLoc'] = []
        else:
            self.clickCache['mapLoc'].append([x, y])
            self.obsPair = True

    def updateClickCacheAndEdges(self, x, y, waypointIndex, button):
        self.clickCache['mapLoc'].append([x, y, waypointIndex])
        if len(self.clickCache['mapLoc']) == 2:  # pair inside
            if self.clickCache['mapLoc'][0][2] == self.clickCache['mapLoc'][1][2]:
                # selected same point, means want to delete point, clear cache
                self.removeWaypoint(self.clickCache['mapLoc'][0][2])
                self.clickCache['mapLoc'] = []
            else:
                self.editEdge(button)

    def onClick(self, button, mapX, mapY, displayX, displayY):
        """reacts to mouse clicking"""

        if mapX is not None and button != 2 and self.config.activeClick and (displayX > self.config.graphSpace[0] and displayX < self.config.graphSpace[1]) and (displayY > self.config.graphSpace[2] and displayY < self.config.graphSpace[3]):

            if self.dragObstacle(mapX, mapY):
                pass

            #add start waypoint if needed
            elif self.waypointTypeStart:
                self.addStartWaypoint(mapX, mapY)

            elif self.waypointNotObs:
                waypointIndex = self.tooClose(mapX, mapY)
                if waypointIndex != -1: #too close to another waypoint so is selecting it
                    self.updateClickCacheAndEdges(mapX, mapY, waypointIndex, button)
                else:
                    #add waypoint to list of waypoints depending on button click
                    self.addWayPoint(mapX, mapY)
            else:
                self.editObstacles(mapX, mapY, button)

            self.buttonUpdate()

    def removeEdgesOfWaypoint(self, waypoint):
        """removes all edges of a specific waypoint"""
        # waypoint removed, now remove all edges connected to it in edge list
        remEdges = []
        for edge in self.map.edges:
            if waypoint == edge[0] or waypoint == edge[1]:
                remEdges.append(edge)
        # now delete edges, this is done this way as the list changes size as the edges are deleted so cannot iterate though list
        for edge in remEdges:
            self.map.edges.remove(edge)

    def removeWaypoint(self, waypoint):
        """removes the waypoint, shifts the indexes of the waypoints, and removes all waypoint edges"""
        if waypoint == 0:
            print('Cannot remove start point, can only move it')
            return
        #since we are using a dictionary, if we do it out of order, then we can't check if the map converted correctly
        #so, we will swap the waypoint with the last waypoint
        self.removeEdgesOfWaypoint(waypoint)
        #now need to set the last waypoint to where the previous waypoint was
        #cannot use self.waypoint counts since intentionally skips over 0 so that the start point is always placed
        ifCase = False #made since each if and elif has the exact same code

        if 0 in self.map.waypoints:
            if len(self.map.waypoints) == 2:
                # no other waypoints, can delete safely
                ifCase = True
            else:
                self.swapAndDeleteWaypoint(waypoint)
        elif len(self.map.waypoints) == 1:
            # no other waypoints, can delete safely
            ifCase = True
        elif waypoint == self.waypointCount+1:
            #remove last waypoint placed
            ifCase = True
        else:
            self.swapAndDeleteWaypoint(waypoint)

        if ifCase:
            self.map.waypoints.pop(waypoint)
            self.waypointCount -= 1

    def swapAndDeleteWaypoint(self, waypoint):
        """swaps last waypoint with waypoint"""
        self.map.waypoints[waypoint] = self.map.waypoints[
            self.waypointCount]  # this is the last index
        self.map.waypoints.pop(self.waypointCount)
        # now redo its edges
        for i in range(0, len(self.map.edges)):
            edge = self.map.edges[i]
            if self.waypointCount == edge[0]:
                self.map.edges[i][0] = waypoint
            elif self.waypointCount == edge[1]:
                self.map.edges[i][1] = waypoint
        self.waypointCount -= 1

    def inObs(self, x, y):
        """determines if a point is in a obstacle"""
        for obs in self.map.obstacles:
            if x <= obs[0] + obs[2]/2 and x >= obs[0] - obs[2]/2:
                if y <= obs[1] + obs[3]/2 and y >= obs[1] - obs[3]/2:
                    return True, obs
        return False, []

    def makeObFromCorner(self):
        """make an obstacle from corners in clickCache"""
        c1 = self.clickCache['mapLoc'][0]
        c2 = self.clickCache['mapLoc'][1]
        obs = toolbox.rectangleFromTwoPoints(c1, c2)
        return obs

    def tooClose(self, xM, yM):
        """determines if a new waypoint is close to another waypoint, if it is, returns which waypoint it is next to"""
        shortest = 999999
        shortestWaypoint = 0
        for waypoint in self.map.waypoints:
            x = xM-self.map.waypoints[waypoint][0]
            y = yM-self.map.waypoints[waypoint][1]
            distance = np.linalg.norm([x, y])
            if distance <= self.config.nearPoint and distance < shortest:
                shortest = distance
                shortestWaypoint = waypoint
        if shortest != 999999:
            return shortestWaypoint
        return -1

    def checkIfEdge(self, newEdge):
        """check if a new edge to be added or removed is in the edge list"""
        if newEdge in self.map.edges:
            return 1
        newEdge = newEdge[::-1]
        if newEdge in self.map.edges:
            return -1
        return 0

path = toolbox.generateFolder('wayPointGenTesting', False)
a = waypointGen(SimMainConfigClass.waypointGenerationConfig, SimMainConfigClass.waypointGenerationDisplayConfig, path,
                'simulationAssets/ShellMapFinal.json', '', 'simulationAssets/odom_data.xls')#shellMap.png, #testingForBugs_0.json
#EDGEMAP.json
#odom_data.xls
#odom_data_reverse.xls
#odom_data_heading_fix.xls
a.displayFullSimulation()
