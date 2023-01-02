import time
import math
import numpy as np
import MiniROS
import toolbox

from controlSystems import routePlanner
import SimMainConfigClass
import simulationBaseClass
import MainConfigClass
from controlSystems.routePlanner import *
import ConfigLoader

"""Make it possible to test attraction, repelling
visual system for attraction and repulsion"""

'''
Take in a map, path []
Take in initial condition/state
run the waypointManager for current path
run the potential field for each point of the vehicle
[possiblity/framework for dynamic]
show in graph car
'''


class simSandbox(simulationBaseClass.simulationInterface):

    def __init__(self, simulationConfig, displayConfig, maps, path, carX, carY, initialVelocity, throttle, brake, steering, gear, handbrake, heading):

        super(simSandbox, self).__init__(simulationConfig, displayConfig, maps, path)

        # set up intial car state
        self.carState = carState(carX, carY, initialVelocity, throttle, brake,
                                 steering, gear, handbrake, heading)

        self.ROS = MiniROS.miniRos(self.carState, self.map.waypoints, self.map.edges)
        self.ROS.vehicleControls.config.stuckDetection = False

        self.setup(self.config.currentSenario, self.config.obstacleWidth, self.config.obstacleHeight)

    def setup(self, senario, obsWidth, obsHeight):
        """prepare the simulation environment"""

        obstaclesOrLIDAR = [] #x,y,w,h

        try:
            senarioData = self.config.senarios[senario]
        except:
            senarioData = self.config.baseSenario

        if self.ROS.potentialField.config.vision:
            #obstacle format
            ending = [obsWidth, obsHeight]
        else:
            #LIDAR format
            ending = [obsHeight]

        for obs in senarioData['obstacles']:
            obstaclesOrLIDAR.append(obs+ending)

        self.environmentState = environmentState(obstaclesOrLIDAR, senarioData['goal'],
                                                 senarioData['rayVector'],
                                                 self.ROS.potentialField.config.vision)
        self.ROS.currentGoal = self.environmentState.goal
        self.ROS.updateROSVars(self.environmentState.obstaclesOrLIDAR, self.environmentState.rayVector, self.carState.position, self.carState.heading)
        self.ROS.updatePotentialField()
        self.directionVector = self.ROS.directionVector[:]
        self.ROS.updateVehicleControls()


        print("Environment setup complete")

    def updateEnvironment(self, change):
        """updates the simulation environment (obstacles, vehicle, etc)"""
        self.environmentState.updateObstaclesOrLIDAR(change)
        self.environmentState.updateRayVector(change)
        self.environmentState.updateGoal(change)

    def runSimulationCycle(self):
        """runs the simulation cycle once"""

        #both of these separated so can be individually run and checked with displaySimulation
        self.ROS.updateROSVars(self.environmentState.obstaclesOrLIDAR,
                           self.environmentState.rayVector,
                           self.carState.position, self.carState.heading)

        #TODO change if needed
        self.ROS.currentGoal = self.environmentState.goal
        self.ROS.updatePotentialField()
        self.ROS.updateVehicleControls()

        #determine car movement
        self.carState.updatePosition(self.ROS.vehicleControls.currentThrottle,
                                     self.ROS.vehicleControls.currentSteering,
                                     self.ROS.vehicleControls.currentBrake)
        change = self.carState.position
        change = [change[0]*-1, change[1]*-1] #flipped the direction to shift the obstacles in relation to the vehicle

        #shift environment in relation to car
        self.updateEnvironment(change)
        self.carState.position = [0,0] #set to 0 since now environment has shifted around car

    def populateGUI(self):
        """update the GUIs from the config, can also be used to update the intial value during a run"""

        #set up buttons with functions
        #changeMode
        self.config.buttons['changeMode']['function'] = self.changeModeButton
        #update vechicle controls
        self.config.buttons['updateVehicleControls']['function'] = self.updateVehicleControlsButton

        #set up sliders with functions
        #Rep
        self.config.sliders['Rep']['function'] = self.repellingSlider
        self.config.sliders['Rep']['initialValue'] = self.ROS.potentialField.config.repellingFactor

        #RepMult
        self.config.sliders['RepMult']['function'] = self.repellingMultipleSlider
        self.config.sliders['RepMult']['initialValue'] = self.ROS.potentialField.config.repellingScalingParameter

        #Atrac
        self.config.sliders['Attrac']['function'] = self.attractionSlider
        self.config.sliders['Attrac']['initialValue'] = self.ROS.potentialField.config.attractingFactor

        #max potential
        self.config.sliders['MaxPotential']['function'] = self.maxPotentialSlider
        self.config.sliders['MaxPotential']['initialValue'] = self.ROS.potentialField.config.maxPotentialAbs

        #velocity proportion
        self.config.sliders['velocityProportion']['function'] = self.velocityProportionSlider
        self.config.sliders['velocityProportion']['initialValue'] = self.ROS.vehicleControls.config.throttleProportion

        #velocity integral
        self.config.sliders['velocityIntegral']['function'] = self.velocityIntegralSlider
        self.config.sliders['velocityIntegral']['initialValue'] = self.ROS.vehicleControls.config.throttleIntegral

        #velocity derivative
        self.config.sliders['velocityDerivative']['function'] = self.velocityDerivativeSlider
        self.config.sliders['velocityDerivative']['initialValue'] = self.ROS.vehicleControls.config.throttleDerivative

        #add to plot
        self.display.loadGUI(self.config.buttons, self.config.sliders, self.config.textBoxes)

    def populateGraph(self):
        """set up objects - rectangles, circles, and arrows - and then populate the graph with those objects"""

        #set up rectangles

        rectangles = [[[self.ROS.potentialField.config.car], ['black', False]]]

        #point location, radius, type
        circles = [[[self.environmentState.goal], self.ROS.waypointControl.config.goalRadius, ['green', True]],
                   [[[0,0]], self.ROS.potentialField.inputHandler.config.goalCircleRadius, ['red', False]],
                   [[[0,0]], self.ROS.potentialField.config.relevantDistance, ['blue', False]],
                   [[[0,0]], self.ROS.potentialField.config.maxPotentialAbs, ['black', False]],
                   [[self.ROS.potentialField.goal], .3, ['green', True]]]

        print(self.directionVector)
        print("magnitude: ", str(toolbox.fastNpLinAlg(self.directionVector)))

        arrows = [[0,0,self.directionVector[0], self.directionVector[1]]]

        #LIDAR or obstacle format, now both are the same so no problem
        if self.ROS.potentialField.config.vision:
            if self.environmentState.rayVector != [] and self.ROS.potentialField.config.rayVector:
                arrows.append([self.environmentState.rayVector[0], self.environmentState.rayVector[1], self.environmentState.rayVector[0]*-1, self.environmentState.rayVector[1]*-1])
            else:
                rectangles.append([self.environmentState.obstaclesOrLIDAR, ['blue', True]])
        else:
            self.ROS.potentialField.inputHandler.cloudPoints = self.environmentState.obstaclesOrLIDAR
            obstacles = self.ROS.potentialField.inputHandler.convertLIDARToObstacles(.2, .2)
            rectangles.append([obstacles, ['blue', True]])

        #add to plot
        self.display.plotShapes(rectangles, circles, arrows)

    def changeModeButton(self, event):
        """changeMode button function"""

        self.config.nextMode()
        self.setup(self.config.currentSenario, self.config.obstacleWidth, self.config.obstacleHeight)
        self.sliderUpdate()

    def updateVehicleControlsButton(self, event):
        """updateVehicleControls button function"""

        self.config.updateVehicleControls = not self.config.updateVehicleControls
        self.sliderUpdate()

    def attractionSlider(self, value):
        """change attraction variable"""

        #get value
        attraction = self.display.sliders[self.config.sliders['Attrac']['name']].val
        self.ROS.potentialField.config.attractingFactor = attraction
        if self.ROS.potentialField.currentGear == MainConfigClass.FORWARD:
            self.ROS.potentialField.config.attractingForwardFactor = attraction
        else:
            self.ROS.potentialField.config.attractingReverseFactor = attraction

        self.sliderUpdate()

    def repellingSlider(self, val):
        """change repelling variable"""

        #get value
        repelling = self.display.sliders[self.config.sliders['Rep']['name']].val
        self.ROS.potentialField.config.repellingFactor = repelling

        self.sliderUpdate()

    def repellingMultipleSlider(self, val):
        """change repelling multiple variable"""

        #get value
        repellingScalingParameter = self.display.sliders[self.config.sliders['RepMult']['name']].val
        self.ROS.potentialField.config.repellingScalingParameter = repellingScalingParameter

        self.sliderUpdate()

    def maxPotentialSlider(self, val):
        """change repelling multiple variable"""

        #get value
        maxPotential = self.display.sliders[self.config.sliders['MaxPotential']['name']].val
        self.ROS.potentialField.config.maxPotential = maxPotential
        self.ROS.potentialField.config.maxPotentialAbs = abs(maxPotential)

        self.sliderUpdate()

    def velocityProportionSlider(self, val):
        """change velocity proportion multiple variable"""

        #get value
        velocityProportion = self.display.sliders[self.config.sliders['velocityProportion']['name']].val
        self.ROS.vehicleControls.config.throttleProportion = velocityProportion

        self.sliderUpdate()

    def velocityIntegralSlider(self, val):
        """change velocity integral variable"""

        #get value
        throttleIntegral = self.display.sliders[self.config.sliders['velocityIntegral']['name']].val
        self.ROS.vehicleControls.config.throttleIntegral = throttleIntegral

        self.sliderUpdate()

    def velocityDerivativeSlider(self, val):
        """change velocity derivative variable"""

        #get value
        throttleDerivative = self.display.sliders[self.config.sliders['velocityDerivative']['name']].val
        self.ROS.vehicleControls.config.throttleDerivative = throttleDerivative

        self.sliderUpdate()

    def sliderUpdate(self):
        """update the slider's changes"""

        #update potentialFieldMap
        self.ROS.currentGoal = self.environmentState.goal
        self.ROS.updateROSVars(self.environmentState.obstaclesOrLIDAR, self.environmentState.rayVector, self.carState.position, self.carState.heading)
        self.ROS.updatePotentialField()
        self.directionVector = self.ROS.directionVector[:]

        if self.config.updateVehicleControls:
            self.ROS.vehicleControls.velocityIntegrator = 0
            self.ROS.vehicleControls.velocityDerivator = 0
            self.ROS.vehicleControls.velocityPrevious = 0
            print('Vehicle Controls reset')
        self.ROS.updateVehicleControls()
        self.carState.showCarInfo()

        #update graph
        self.display.loadWindow(self.config.windowSize, self.config.legend, self.config.title)
        self.populateGraph()

    def runDynamicSimulation(self):#############
        """runs the full simulation continously until the car reaches the goal"""
        print('Simulation started')
        start = time.time()
        timeSpent = 0
        goalReached = False
        
        print('Running...')
        while goalReached == False:
            print(str(timeSpent), '...')
            self.runSimulationCycle()
            timeSpent += self.config.timeChange

            goalReached = self.ROS.waypointControl.goalCheck(self.carState.position, self.environmentState.goal) #self.carState needs to be [0,0] for it to work, need to use this for the change before
            if goalReached:
                print('Goal has been reached')

            if self.config.showSim == True:
                self.displaySimulation()

            #timeout check
            if timeSpent > 10:
                print("Was not able to get to goal")
                break

        end = time.time()
        #simulation has ended
        print('Simulation time', str(timeSpent), 'with', str(self.ROS.potentialField.collision), 'collision')
        print('Simulation run took', str(end-start))

    def outputChanges(self):
        """Output a copy of the config"""
        ConfigLoader.saveJSON(self.ROS.potentialField.config, "potentialFieldMap config", "generate the potentialFieldMap config from the attraction repulsion sim")
        ConfigLoader.saveJSON(self.ROS.vehicleControl.config, "vehicleControl config", "generate the vehicleControl config from the attraction repulsion sim")
        ConfigLoader.saveJSON(self.ROS.waypointControl.config, "goalControl config", "generate the goalControl config from the attraction repulsion sim")

class carState():
#class that defines state of vehicle

    def __init__(self, carX, carY, initialVelocity, throttle, brake, steering, gear, handbrake, heading):
        """set up car state"""

        self.position = np.array([carX, carY])
        self.velocity = initialVelocity
        self.throttle = throttle
        self.brake = brake
        self.steering = steering
        self.gear = gear
        self.handbrake = handbrake
        self.heading = heading

    def showCarInfo(self):
        """show the current controls and velocity"""
        print('Throttle: ', self.throttle)
        print('Steering: ', self.steering)
        print('Brake: ', self.brake)
        print('Velocity: ', self.velocity)
        print()

    def updatePosition(self, throttle, steering, brake):####################
        """update the car's position using dynamics and what throttle, steering, and brake mean (throttle force, brake force) 225 kg"""
        self.throttle = throttle
        self.brake = brake
        self.steering = steering

        #now calculate how the position is changed
        calculatedPosition = [2,2]

        self.position = calculatedPosition

    def save(self):
        """save the class state"""
        ConfigLoader.saveJSON(self, "vehicleState", "generate the current vehicle state")

class environmentState():
#class that defines environment of the simulation

    def __init__(self, obstaclesOrLIDAR, goal, rayVector, vision):
        
        #set up obstacles
        self.obstaclesOrLIDAR = obstaclesOrLIDAR
        self.vision = vision
        self.rayVector = rayVector

        if self.vision:
            #link the obstacles together by count, their ID, could also do based off position in list (and is faster), count included just in case
            count = 0
            for obstacle in self.obstaclesOrLIDAR:
                count += 1
                obstacle.append(count)
            self.obstacles = np.asarray(self.obstaclesOrLIDAR)
            #back up of size for when rotating the obstacle
            self.origionalObstacles = self.obstacles

        self.goal = goal

    def updateObstaclesOrLIDAR(self, change):
        """updates the updateObstaclesOrLidars' position"""

        for thing in self.obstaclesOrLIDAR:
            thing[0] = thing[0] + change[0]
            thing[1] = thing[1] + change[1]

        if self.vision:
            pass
            #see if can rotate obstacles and draw box around new edges for greater accuracy when turning

    def updateRayVector(self, change):
        """updates the rayVector's position if it is present"""
        if self.rayVector != []:
            self.rayVector[0] += change[0]
            self.rayVector[1] += change[1]


    def updateGoal(self, change):
        """updates the goal's position"""
        self.goal[0] =  self.goal[0] + change[0]
        self.goal[1] =  self.goal[1] + change[1]

    def save(self):
        """save the class state"""
        #convert to non-numpy formats

        if self.vision:
            #modify obstacles
            tempObs = self.obstaclesOrLIDAR
            self.obstaclesOrLIDAR = self.obstaclesOrLIDAR.tolist()
            for obs in self.obstaclesOrLIDAR:
                obs = obs[0:5]

        #modify goal
        tempGoal = self.goal
        self.goal = self.goal.tolist()

        ConfigLoader.saveJSON(self, "environment", "generate the current environment state")

        #revert to regular values
        if self.vision:
            self.obstaclesOrLIDAR = tempObs
        self.goal = tempGoal

path = toolbox.generateFolder('carCalibration', False)
sim = simSandbox(SimMainConfigClass.carCalibrationSim, SimMainConfigClass.carCalibrationSimDisplayConfig, 'simulationAssets/ShellMapFinal.json', path, 0, 0, 0, 0, 0, 0, MainConfigClass.FORWARD, False, [0, 1])

sim.ROS.updatePotentialField()
sim.ROS.updateVehicleControls()
sim.carState.showCarInfo()
sim.displayFullSimulation()
