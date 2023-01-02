import simulationBaseClass
import SimMainConfigClass
import toolbox
import random
from controlSystems import routePlanner

class BMGSim(simulationBaseClass.simulationInterface):

    def __init__(self, simulationConfig, displayConfig, maps, path):
        super(BMGSim, self).__init__(simulationConfig, displayConfig, maps, path)


        self.mapUndecided = False

        self.setup(maps)

    def setup(self, maps):
        """prepare the simulation environment"""
        print("Environment setup complete")
        if maps == '':
            #no map given
            self.maps = []
            self.generateNewMap(5)
            self.map = self.maps[0]

    def populateGUI(self):
        """update the GUIs from the config, can also be used to update the intial value during a run"""

        self.populateBaseGUI()

        # set up buttons with functions
        self.config.buttons['accept']['function'] = self.acceptMap

        self.config.buttons['reject']['function'] = self.rejectMap

        self.config.buttons['generate']['function'] = self.generateNewMap

        self.config.buttons['last']['function'] = self.goToLast

        # add to plot
        self.display.loadGUI(self.config.buttons, self.config.sliders, self.config.textBoxes)

    def populateGraph(self):
        """set up objects - rectangles, circles, and arrows - and then populate the graph with those objects"""

        rectangles, circles, arrows = self.populateBaseGraph()

        self.display.plotShapes(rectangles, circles, arrows)

    def goToLast(self, event):
        """goes to the last map"""
        self.mapIndex = 0
        self.previousMap(5)

    def acceptMap(self, event):
        """pause button function"""
        if self.mapUndecided:
            self.mapUndecided = False
            print('Map accepted')
        else:
            print('A map has not been generated')

    def rejectMap(self, event):
        """reject or delete a map"""
        if self.mapUndecided:
            self.mapUndecided = False
            self.maps.pop()
            if len(self.maps) == 0:
                self.maps.append(routePlanner.autoMap())
            self.previousMap(1)
            print('Map rejected')
        else:
            print('A map has not been generated')
            if input('Press 1 if you want to delete the map anyway:\n') == '1':
                self.maps.pop(self.mapIndex)
                if len(self.maps) == 0:
                    self.maps.append(routePlanner.autoMap())
                print('Map removed')
                self.previousMap()
        self.buttonUpdate()

    def generateNewMap(self, event):
        """generates obstacles based on map size"""
        # with obstacleDenominator use low numbers with caution, 10 and 15 are medium density

        if self.mapUndecided:
            print('You did not accept or reject your previous map, assuming accepted map')

        max_obstacles = ((self.config.windowSize[3] + self.config.windowSize[2]) // self.config.obstacleDenominator * 2)

        obstacles = []

        while max_obstacles >= 0:  # generate obstacles
            # random shape sizes
            if self.config.randomWH:
                rectangleWidth = random.randint(2, 5)
                rectangleHeight = random.randint(2, 5)
            else:
                rectangleWidth = self.config.baseObsWidth
                rectangleHeight = self.config.baseObsHeight

            x = random.randint(self.config.windowSize[0] - self.config.windowSize[2] / 2, self.config.windowSize[0] + self.config.windowSize[2] / 2 - 1)
            y = random.randint(self.config.windowSize[1] - self.config.windowSize[3] / 2, self.config.windowSize[1] + self.config.windowSize[3] / 2 - 1)

            obstacle = [x, y, rectangleWidth, rectangleHeight]

            if toolbox.getDistance(obstacle, self.config.protectedArea[0:2], self.config.protectedArea[2], self.config.protectedArea[3]) != [0, 0]:
                max_obstacles -= 1
                obstacles.append(obstacle)

        self.maps.append(routePlanner.autoMap({}, obstacles, [], []))
        print('Map', len(self.maps), 'Generated')
        if event != 5:
            self.mapUndecided = True
            self.mapIndex = 0
            self.previousMap(5)

path = toolbox.generateFolder('bulkMapGenerationTesting', False)
bmgSim = BMGSim(SimMainConfigClass.BMGSimConfig, SimMainConfigClass.BMGSimDisplayConfig, 'simulationAssets/45Maps.json', path)
bmgSim.displayFullSimulation()