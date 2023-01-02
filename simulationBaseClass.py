import abc
import displaySimTools
import ConfigLoader
import controlSystems.routePlanner
from controlSystems.routePlanner import *
from MainConfigClass import *

class simulationInterface(metaclass=abc.ABCMeta):
    @classmethod
    def __subclasshook__(cls, subclass):
        return (hasattr(subclass, 'setup') and
                callable(subclass.setup) and
                hasattr(subclass, 'populateGUI') and
                callable(subclass.populateGUI) and
                hasattr(subclass, 'populateGraph') and
                callable(subclass.populateGraph) or
                NotImplemented)

    def __init__(self, simulationConfig, displayConfig, maps = '', path = ''):
        self.config = simulationConfig()
        displayConfigEditor = displayConfig()
        displayConfigEditor.clickFunction = self.onClick
        displayConfigEditor.moveFunction = self.onMove
        displayConfigEditor.updateClickArea = self.userResized
        self.display = displaySimTools.displayTools(displayConfigEditor)

        self.dragging = False
        self.storedObInd = -1

        self.path = path

        self.viewConverted = False

        self.rectangleData = []
        self.circleData = []
        self.arrowData = []

        self.mapName = maps
        if self.mapName == '':
            self.prepareMaps([], self.config.shiftX, self.config.shiftY)
        else:
            rawMaps = ConfigLoader.loadJSON(self.mapName)
            self.prepareMaps(rawMaps, self.config.shiftX, self.config.shiftY)

    def prepareMaps(self, maps, shiftX, shiftY):
        """process the maps inputted"""

        if type(maps) == list:
            #multiple maps

            if len(maps) == 0:
                #no maps, add a blank map
                newMap = routePlanner.autoMap()
                self.maps = [newMap]

            else:
                for i in range(0, len(maps)):
                    newMap = self.processMap(maps[i], shiftX, shiftY)
                    maps[i] = newMap
                self.maps = maps
        else:
            self.maps = [self.processMap(maps, shiftX, shiftY)]

        self.mapIndex = 0
        self.map = self.maps[self.mapIndex]


    def processMap(self, singleMap, shiftX, shiftY):
        """process a single map, adding the shiftX and shiftY and preparing dictionary indexes"""

        tempWaypoints = {}
        for key in singleMap.waypoints:
            #shifts and makes the keys integers
            tempWaypoints[int(key)] = [singleMap.waypoints[key][0] + shiftX,
                                       singleMap.waypoints[key][1] + shiftY]

        singleMap.waypoints = tempWaypoints

        for goal in range(0, len(singleMap.goals)):
            singleMap.goals[goal][0] = singleMap.goals[goal][0] + shiftX
            singleMap.goals[goal][1] = singleMap.goals[goal][1] + shiftY

        for b in range(0, len(singleMap.obstacles)):
            singleMap.obstacles[b][0] = singleMap.obstacles[b][0] + shiftX
            singleMap.obstacles[b][1] = singleMap.obstacles[b][1] + shiftY

        return singleMap

    @abc.abstractmethod
    def setup(self):
        """prepare the simulation environment"""
        print("Environment setup complete")
        raise NotImplementedError

    @abc.abstractmethod
    def populateGUI(self):
        """update the GUIs from the config, can also be used to update the initial value during a run"""
        #self.display.loadGUI(self.config.buttons, self.config.sliders, self.config.textBoxes)
        raise NotImplementedError

    @abc.abstractmethod
    def populateGraph(self):
        """set up objects - rectangles, circles, and arrows - and then populate the graph with those objects"""
        raise NotImplementedError

    def populateBaseGUI(self):
        """populates the base GUI elements"""

        # add debug functionality
        self.config.buttons['debug']['function'] = self.debug

        # add click toggle functionality
        self.config.buttons['toggleClick']['function'] = self.toggleClick

        # add drag toggle functionality
        self.config.buttons['toggleDrag']['function'] = self.toggleDrag

        # add randomDebug button functionality
        self.config.buttons['randomDebug']['function'] = self.randomDebug

        # add button to reset plot
        self.config.buttons['reset']['function'] = self.reset

        # add button to check map conversion
        self.config.buttons['viewConverted']['function'] = self.toggleConversion

        # export map file to JSON
        self.config.buttons['exportToJSON']['function'] = self.exportToJSON

       # cycle to next map
        self.config.buttons['nextMap']['function'] = self.nextMap

        # cycle to previous map
        self.config.buttons['previousMap']['function'] = self.previousMap

    def populateFromProcessedList(self):
        nodes, paths, obs = getGraphFromMapJSON(self.map)  # its a list of maps
        points = []
        edges = []
        for node in nodes:
            points.append([node.x, node.y])

        for path in paths:
            nodeOne = [path.nodes[0].x, path.nodes[0].y]
            nodeTwo = [path.nodes[1].x, path.nodes[1].y]

            edges.append([points.index(nodeOne), points.index(nodeTwo)])

        print('Successful conversion, nodes and edges extracted')
        return points, edges, obs

    def populateBaseGraph(self):
        """populates the map elements"""

        if self.viewConverted:
            alternatePoints, alternateEdges, obs = self.populateFromProcessedList()
            waypoints = alternatePoints
        else:
            obs = self.map.obstacles
            waypoints = self.map.waypoints.values()

        # RECTANGLES
        rectangles = [[obs, ['blue', True]]]

        # CIRCLES
        circles = [[self.map.goals, self.config.goalRadius + self.config.goalBoostRadius, ['green', True]],
                   [waypoints, self.config.waypointRadius, ['black', True]]]

        if 0 in self.map.waypoints:
            if self.viewConverted:
                startPoint = alternatePoints[0]
            else:
                startPoint = [self.map.waypoints[0][0], self.map.waypoints[0][1]]

            circles.append([[startPoint], self.config.waypointRadius, ['red', True]])

        # ARROWS
        arrows = []

        if self.viewConverted:
            edges = alternateEdges
            waypoints = alternatePoints
        else:
            edges = self.map.edges
            waypoints = self.map.waypoints
        for edge in edges:
            pointOne = waypoints[edge[0]]
            pointTwo = waypoints[edge[1]]
            arrows.append([pointOne[0], pointOne[1], pointTwo[0] - pointOne[0], pointTwo[1] - pointOne[1]])

        # extra things that need to be plotted
        if self.circleData != []:
            circles += self.circleData
            self.circleData = []
        if self.rectangleData != []:
            rectangles += self.rectangleData
            self.rectangleData = []
        if self.arrowData != []:
            arrows += self.arrowData
            self.arrowData = []

        return rectangles, circles, arrows

    def displayFullSimulation(self):
        """displays the current simulation environment that is continuously updated if GUI elements are present"""

        # create the plot
        self.display.setUpPlot(self.config.adjustBottom)

        # load the window
        self.display.loadWindow(self.config.windowSize, self.config.legend, self.config.title)

        # add GUI elements
        self.populateGUI()

        # add graph elements
        self.populateGraph()

        # updates graph clickable area
        self.display.userResized(1)

        # show plot
        self.display.showPlot()

    def buttonUpdate(self):
        """update the button's changes"""
        # update graph
        self.display.loadWindow(self.config.windowSize, self.config.legend, self.config.title)
        self.populateGraph()

    def debug(self, event):
        """a place to activate any functions to run print statements or a safe place to check the state of the map"""
        print()
        print('-----------------------------------------------------------------')
        print('Debug Values')
        print('-----------------------------------------------------------------')
        print()
        classData = self.__dict__
        for var in classData:
            item = classData[var]
            if hasattr(item, '__dict__') and var not in self.config.ignoredDebug:
                print(item)
                levelTwo = item.__dict__
                for varTwo in levelTwo:
                    print(varTwo, levelTwo[varTwo])
            else:
                print(var, item)

        print()
        print('-----------------------------------------------------------------')
        print('Debug End')
        print('-----------------------------------------------------------------')
        print()

    def toggleClick(self, event):
        """toggles click and off"""
        self.config.activeClick = not self.config.activeClick
        if self.config.activeClick:
            print("Clicking Active")
        else:
            print("Clicking Not Active")

    def toggleDrag(self, event):
        self.config.activeDrag = not self.config.activeDrag
        if self.config.activeDrag:
            print("Dragging Active")
        else:
            print("Dragging Not Active")

    def reset(self, event):
        """resets the state of everything in the plot"""
        if input('Are you sure? 1 yes, any other key no: ') == '1':
            print()
            if self.mapName == '':
                self.prepareMaps([], self.config.shiftX, self.config.shiftY)
            else:
                rawMaps = ConfigLoader.loadJSON(self.mapName)
                self.prepareMaps(rawMaps, self.config.shiftX, self.config.shiftY)
            print('System reset')
            self.buttonUpdate()

    def randomDebug(self, event):
        """place to put random functions to test and debug without having to make a new button for it immediately"""
        print('This is the randomDebug, you need to override it to have your own random debug')


    def inObsDrag(self, x, y):
        """determines if a point is in a obstacle"""

        for i,obs in enumerate(self.map.obstacles):
            if x <= obs[0] + obs[2]/2 and x >= obs[0] - obs[2]/2:
                if y <= obs[1] + obs[3]/2 and y >= obs[1] - obs[3]/2:
                    return i
        return -1

    def onMove(self, mapX, mapY, displayX, displayY):
        if self.dragging and mapX is not None and self.config.activeClick and (
                    displayX > self.config.graphSpace[0] and displayX < self.config.graphSpace[1]) and (
                    displayY > self.config.graphSpace[2] and displayY < self.config.graphSpace[3]):
            self.map.obstacles[self.storedObInd][0] = mapX
            self.map.obstacles[self.storedObInd][1] = mapY
            self.buttonUpdate()

    def dragObstacle(self, mapX, mapY):
        """logic for moving obstacles"""
        if self.config.activeDrag:
            if not self.dragging:
                obInd = self.inObsDrag(mapX, mapY)
                if not obInd == -1:
                    self.storedObInd = obInd
                    self.dragging = True
                else:
                    print("Please click on an obstacle.")
            else:
                self.storedObInd = -1
                self.dragging = False
            return True
        return False

    def onClick(self, button, mapX, mapY, displayX, displayY):
        """reacts to mouse clicking"""
        if mapX is not None and button != 2 and self.config.activeClick and (displayX > self.config.graphSpace[0] and displayX < self.config.graphSpace[1]) and (displayY > self.config.graphSpace[2] and displayY < self.config.graphSpace[3]):
            # print('Inside Graph area')

            self.dragObstacle(mapX, mapY)

        # print('This is the clicking function, you need to override it to have your own random debug')
        # print('Button', button)
        # print('MapX', mapX)
        # print('MapY', mapY)
        # print('DisplayX', displayX)
        # print('DisplayY', displayY)

    def userResized(self, xWidthSmall, xWidthBig, yWidthSmall, yWidthBig):
        """reset the boundaries for what is and is not clickable"""

        self.config.graphSpace[0] = xWidthSmall
        self.config.graphSpace[1] = xWidthBig
        self.config.graphSpace[2] = yWidthSmall
        self.config.graphSpace[3] = yWidthBig

    def toggleConversion(self, event):
        """toggles converted map on and off"""
        self.viewConverted = not self.viewConverted
        if self.viewConverted:
            print("Converted Map Mode Active")
        else:
            print("Standard Map Mode Active")
        self.buttonUpdate()

    def exportToJSON(self, value):
        """exports the map file as a JSON file, asks for name in console"""
        name = input('Name for new map set: ' )

        self.prepareMaps(self.maps, -self.config.shiftX, -self.config.shiftY)

        ConfigLoader.saveJSON(self.maps, name, 'Saving all maps from waypoint generator', self.path)
        print('You must rerun this code with the new file and with the last parameter as false to get the new Shell map data and put it into the MasterClass otherwise NO CHANGES WILL BE SAVED!!!!')
        input('Ok? ')
        for autoMap in self.maps:
            autoMap.showMap()

        self.prepareMaps(self.maps, self.config.shiftX, self.config.shiftY)

    def nextMap(self, event):
        """cycles to the next map"""
        self.mapIndex += 1
        if self.mapIndex == len(self.maps):
            self.mapIndex = 0
        self.map = self.maps[self.mapIndex]
        self.buttonUpdate()
        print('Next map shown')

    def previousMap(self, event):
        """cycles to the last map, 5 as argument to avoid print statement"""
        self.mapIndex -= 1
        if self.mapIndex < 0:
            self.mapIndex = len(self.maps) - 1
        self.map = self.maps[self.mapIndex]
        self.buttonUpdate()
        if event != 5:
            print('Previous map shown')