import RRT.rrtUtils as rrtUtils
import SimMainConfigClass
import toolbox
import simulationBaseClass
from RRT import rrt
import numpy as np


class RRTSim(simulationBaseClass.simulationInterface):

    def __init__(self, simulationConfig, displayConfig, maps, path):
        super(RRTSim, self).__init__(simulationConfig, displayConfig, maps, path)

        self.rrtGen = rrt.rrtPathGenerator()
        self.setup()

    def setup(self):
        """prepare the simulation environment"""
        print("Environment setup complete")

    def populateGUI(self):
        """update the GUIs from the config, can also be used to update the initial value during a run"""

        self.populateBaseGUI()

        # add to plot
        self.display.loadGUI(self.config.buttons, self.config.sliders, self.config.textBoxes)

    def populateGraph(self):
        """set up objects - rectangles, circles, and arrows - and then populate the graph with those objects"""

        rectangles, circles, arrows = self.populateBaseGraph()

        self.route = self.rrtGen.generateRoute(self.rrtGen.config.SEARCH_WINDOW, np.asarray(self.map.obstacles), np.asarray(self.config.goalRegion))

        for i in range(len(self.route[0])-1):
            x1 = self.route[0][i]
            y1 = self.route[1][i]
            x2 = self.route[0][i+1]
            y2 = self.route[1][i+1]
            arrows.append([x1,y1,x2-x1,y2-y1])

        circles.append([[self.config.goalRegion[:2]], self.config.goalRegion[2], ['green', True]])

        self.display.plotShapes(rectangles, circles, arrows)


path = toolbox.generateFolder('rrtTesting', False)
rrtSim = RRTSim(SimMainConfigClass.RRTSimConfig, SimMainConfigClass.RRTSimDisplayConfig, "simulationAssets/rrtMaps.json", path)
rrtSim.displayFullSimulation()
