#!/usr/bin/env python
import math
import numpy as np

class autoMap:
    def __init__(self, waypoints = {}, obstacles = [], goals = [], edges = []):
        self.waypoints = waypoints
        self.obstacles = obstacles
        self.goals = goals
        self.edges = edges

    def showMap(self):
        """prints map information"""
        print()
        print('Waypoints:', self.waypoints)
        print('Edges:', self.edges)
        print('Obstacles:', self.obstacles)
        print('Goals:', self.goals)
        print()

###################Globals#######################
GLOBAL_LOGFLAG = False
localSim = True

#Car Globals
INITIAL_HEADING = math.pi
START_LOCATION = [0.075, -0.008]
MAX_SPEED = 8.94 #12, 8.94 m/s is 20 mph
IGNORE_VELOCITY = -101010
FORWARD = "forward"
REVERSE = "reverse"

#Navigation Globals
IGNORE_REVERSE = True
GOAL_RADIUS = 3.0
WAYPOINT_DETECTION_DISTANCE = 4.0
GOALS_COMPLETED = "goals_completed"
ANGLE_DIFFERENCE = 1.63 #1.57 is 90 degres but .06 added for little mismatches
DOUBLE_PI_ANGLE_DIFFERENCE = 2*math.pi - ANGLE_DIFFERENCE

#Map Globals
WAYPOINTS = {0: [0.0, 0.0], 1: [-76.13, -0.08], 2: [-91.4, -0.08], 3: [-203.48, -0.08], 4: [-212.06, -9.45],
             5: [-212.06, -39.12], 6: [-212.06, -56.09], 7: [-212.06, -120.14], 8: [-212.06, -136.81],
             9: [-212.06, -246.83], 10: [-203.62, -255.92], 11: [-93.28, -255.92], 12: [-74.71, -255.92],
             13: [34.75, -255.92], 14: [43.94, -246.47], 15: [43.94, -136.0], 16: [43.94, -119.34],
             17: [43.94, -8.68], 18: [34.66, -0.08], 19: [-83.96, -8.95], 20: [-83.96, -40.06],
             21: [-83.96, -56.19], 22: [-83.96, -119.34], 23: [-86.55066931050789, -3.4010641225576137],
             24: [-83.96, -246.78], 25: [36.49, -128.08], 26: [-75.32, -128.08], 27: [-91.86, -128.08],
             28: [-202.48, -128.08], 29: [-92.24, -48.15], 30: [-208.48725946102337, -51.667631057136646],
             31: [-135, 1.5], 32: [-212.0, -73.94999999999999], 33: [-212.0, -198.22], 34: [-135.5, -256.0],
             35: [-84.5, -198.22], 36: [-135.5, -128.0], 37: [-135.5, -48.19999999999999], 38: [44.5, -65.75],
             39: [0.0, -128.0], 40: [44.5, -198.22], 41: [0.0, -256.0], 42: [-208.910650399046, -3.8244550605802488],
             43: [-209.33404133706864, -252.7783266178826], 44: [-41.67122988010982, -127.03121802516355],
             45: [-178.84989379943977, -129.14817271527667], 46: [-84.01032368237213, -135.92242772363866],
             47: [-86.12727837248525, -125.33765427307307], 48: [-86.55066931050789, -131.2651274053898],
             49: [-80.62319617819117, -125.33765427307307], 50: [-80.62319617819117, -131.2651274053898],
             51: [-81.0465871162138, -3.4010641225576137], 52: [-79.7764143021459, -251.50815380381476],
             53: [-83.5869327443495, -224.8345247083895], 54: [-208.48725946102337, -131.68851834341243],
             55: [-208.48725946102337, -125.33765427307307], 56: [-203.48, -48.15],
             57: [-208.48725946102337, -44.893376048774655], 58: [-87.3974511865531, -51.24424011911401],
             59: [-87.3974511865531, -44.893376048774655], 60: [-83.16354180632692, -241.3467712912718],
             61: [40.89000303430174, -131.2651274053898], 62: [40.89000303430174, -124.49087239702783],
             63: [40.04322115825647, -252.35493567986], 64: [40.04322115825647, -3.8244550605802488],
             65: [-87.82084212457573, -251.9315447418374]}
EDGES = [[3, 31], [31, 2], [2, 1], [17, 38], [15, 40], [40, 14], [13, 41], [41, 12], [12, 11], [11, 34], [34, 10], [9, 33], [33, 8], [8, 7], [7, 32], [32, 6], [5, 6], [37, 29], [20, 21], [19, 20], [4, 5], [21, 22], [27, 36], [39, 25], [1, 0], [0, 18], [4, 42], [42, 3], [9, 43], [43, 10], [35, 46], [46, 48], [48, 27], [27, 47], [47, 22], [22, 49], [49, 26], [26, 50], [50, 46], [38, 16], [15, 16], [7, 55], [55, 28], [8, 54], [54, 28], [6, 30], [5, 57], [56, 57], [30, 56], [56, 37], [29, 59], [59, 20], [29, 58], [58, 21], [2, 23], [23, 19], [19, 51], [51, 1], [22, 46], [27, 26], [53, 35], [25, 62], [62, 16], [25, 61], [61, 15], [26, 44], [44, 39], [13, 63], [63, 14], [45, 36], [18, 64], [64, 17], [11, 65], [65, 24], [24, 52], [52, 12], [28, 45], [24, 60], [53, 60]]
OBSTACLES = []

GOALS = [[44.5, -65.75], [0.0, -128.0], [44.5, -198.22], [0.0, -256.0],
         [-135.5, -256.0], [-84.5, -198.22], [-135.5, -128.0],
         [-212.0, -198.22], [-212.0, -73.95], [-135.5, -48.2],
         [-135, 1.5]]

BREADCRUMB = [[0.0, 0.0], [-76.13, -0.08], [-91.4, -0.08], [-135, 1.5], [-135, 1.5], [-203.48, -0.08],
              [-208.910650399046, -3.8244550605802488], [-212.06, -9.45], [-212.06, -39.12],
              [-208.48725946102337, -44.893376048774655], [-203.48, -48.15], [-135.50, -48.20],
              [-203.48, -48.15], [-208.48725946102337, -51.667631057136646], [-212.06, -56.09],
              [-212.0, -73.94999999999999], [-212.06, -120.14], [-208.48725946102337, -125.33765427307307],
              [-208.48725946102337, -131.68851834341243], [-202.48, -128.08], [-178.84989379943977, -129.14817271527667],
              [-135.5, -128.0], [-178.84989379943977, -129.14817271527667], [-202.48, -128.08], [-208.48725946102337, -131.68851834341243],
              [-212.06, -136.81], [-212.0, -198.22], [-212.06, -246.83], [-209.33404133706864, -252.7783266178826],
              [-203.62, -255.92], [-135.5, -256.0], [-93.28, -255.92], [-87.82084212457573, -251.9315447418374],
              [-83.96, -246.78], [-83.16354180632692, -241.3467712912718], [-84.5, -198.22], [-84.01032368237213, -135.92242772363866],
              [-80.62319617819117, -131.2651274053898], [-75.32, -128.08], [-41.67122988010982, -127.03121802516355],
              [0.0, -128.0], [36.49, -128.08], [40.89000303430174, -124.49087239702783], [43.94, -119.34],
              [44.5, -65.75], [43.94, -119.34], [40.89000303430174, -124.49087239702783], [36.49, -128.08],
              [40.89000303430174, -131.2651274053898], [43.94, -136.0], [44.5, -198.22], [43.94, -246.47],
              [40.04322115825647, -252.35493567986], [34.75, -255.92], [0.0, -256.0]]

class baseClass(object):
    def __init__(self):
        self.configInts = []
        self.configLists = []
        self.configBools = []
        self.configDicts = []

    def editConfig(self, setting, value):
        """changes a parameter in the config"""
        if setting in self.configInts and type(value) != int:
            print(setting, 'value must be a integer')
            return
        if setting in self.configLists and type(value) != list:
            print(setting, 'value must be a list')
            return
        if setting in self.configBools and type(value) != bool:
            print(setting, 'value must be a bool')
        if setting in self.configDicts and type(value) != dict:
            print(setting, 'value must be a dictionary')
        self.__dict__[setting] = value

class carConfig(baseClass):
    """config class for the car's information"""

    def __init__(self):
        self.mass = 10
        self.gravity = 9.8
        self.dragC = .002
        self.rollingC = .003
        self.avgVelocity = 4 #optimal
        self.timeLimit = 100
        self.timeLimitWeight = 5
        self.clockSide = True #True means clockwise, False means counter clockwise

class potentialFieldConfig(baseClass):
    """the config class for the potential field"""

    # def __init__(self, jHat : List, relevantDistance : float, repellingFactor : float, attractingFactor : float, maxPotential : float, goalCircleRadius : float, maxRepelling : float,
    #             car : List, configInts : List, configLists : List, LOGFLAG = False, vision):
    def __init__(self):
        self.LOGFLAG = GLOBAL_LOGFLAG or False
        self.sendGoalCompletion = False
        self.vision = True
        self.rayVector = False
        self.jHat = [0, 1]
        self.relevantDistance = 10.0

        self.attractingForwardFactor = .34
        self.attractingReverseFactor = .34
        self.attractingFactor = self.attractingForwardFactor
        self.repellingScalingParameter = 6.07
        self.repellingFactor = 6.84
        self.safetyControllerVelocityFactor = 1.4
        self.maxSpeedFactor = 1 / MAX_SPEED  # is a proprtion, the faster you are going, the more repelling you will have
        self.maxPotential = -1 * MAX_SPEED

        self.maxPotentialAbs = math.fabs(self.maxPotential)
        self.maxRepelling = 1000.0  # perhaps later change so that it only checks when adding with attractive, not when getting repelling

        self.cameraShift = [0.0,2.0]  # used to convert the camera center to car center, is the offset of the LIDAR from the car's center
        self.car = [0, 0, 1.7, 4.2]  # xOffset, yOffset, width, height
        self.initialHeading = [-1.0, 0.0]

        self.ignoreVelocity = IGNORE_VELOCITY

        self.configInts = []  # don't forget to include this one in config lists
        self.configLists = ["configInts", "configLists", "configBool", 'configDicts', "LIDARShift", "car", "jHat",
                            "initialHeading"]  # don't forget to include this one in config lists
        self.configBools = ['LOGFLAG', 'vision']
        self.configDicts = []


class inputProcessingConfig(baseClass):

    def __init__(self):
        self.LIDARShift = [0,0] #used to convert the LIDAR center to car center, is the offset of the LIDAR from the car's center
        self.goalCircleRadius = 6.0
        self.tinyCubeWidth = .01
        self.tinyCubeHeight = .01

        self.configInts = [] #don't forget to include this one in config lists
        self.configLists = ["configInts","configLists", "configBool","LIDARShift"] #don't forget to include this one in config lists
        self.configBools = []
        self.configDicts = []

class goalOverseerConfig(baseClass):

    # def __init__(self, goalRadius : float, goals : List, configInts : List, configLists : List, LOGFLAG = False):
    def __init__(self):
        self.LOGFLAG = GLOBAL_LOGFLAG or False
        self.goalRadius = GOAL_RADIUS

        self.optimalPath = False

        self.waypointRadius = WAYPOINT_DETECTION_DISTANCE
        self.breadcrumbMode = False
        self.goals = GOALS #this will make the code fail, that is why it says potato as a reminder to fill it in
        #TODO get the order of goals that is good
        self.startLocation = START_LOCATION
        self.initialHeading = INITIAL_HEADING

        self.checkIfFoundNonTargetGoal = False

        self.ignoreVelocity = IGNORE_VELOCITY

        self.configInts = []  # don't forget to include this one in config lists
        self.configLists = ["configInts", "configLists", "configBool",
                            'configDicts']  # don't forget to include this one in config lists
        self.configBools = ['LOGFLAG']
        self.configDicts = []

class hybridSplineRouterConfig(baseClass):

    def __init__(self):
        self.LOGFLAG = GLOBAL_LOGFLAG or False
        self.indexJump = 5
        self.frontKnotsCount = 2
        self.backKnotsCount = 2
        self.pivotRadius = 2
        self.targetObstacleWidthMultiplier = 1.5
        self.pivotTries = 200
        #self.frontKnotsDistance = car length will be used


        self.splinePivotPoint = 10
        self.splineGenTries = 170

        self.configInts = ['pivotTries', 'frontKnots', 'backKnots', 'frontKnotTries',
                           'backKnotTries', 'splineTries']  # don't forget to include this one in config lists
        self.configLists = ["configInts", "configLists", "configBool",
                            'configDicts']  # don't forget to include this one in config lists
        self.configBools = ['LOGFLAG']
        self.configDicts = []

class splineSystemsConfig(baseClass):

    def __init__(self):

        self.LOGFLAG = GLOBAL_LOGFLAG or False
        self.look_forward_velocity_gain = 0.1
        self.look_forward_curvature_loss = 1.0
        self.look_forward_distance = 2.0
        self.simulation_time_step = 0.05
        self.max_simulation_time = 100
        self.wheel_base = 1.4
        self.max_steer_rate = 180
        self.vehicle_track = 0.7
        self.max_steer_angle = 45
        self.wheel_radius = self.wheel_base / (math.tan(self.max_steer_angle / 180 * math.pi))
        self.max_curvature = 1 / self.wheel_radius

        self.splineBankOn = False


        self.proportional_value_pure_pursuit = 0.02
        self.integral_value_pure_pursuit = 0.05
        self.derivative_value_pure_pursuit = -0.5
        self.proportional_value_SC = 0.4
        self.step_SC = 0.1
        self.knots = 3
        self.num_splines = 1
        self.num_good_splines = 1
        self.turn_back_tolerance = 1.04
        self.try_generation_limit = 250
        self.splineBankSizePercentage = 0.3
        self.window = [0,17.5,40,45]
        self.max_acceleration = 5
        self.max_jerk = 999999999
        self.discretization_distance = 0.01
        self.granularity = 0.001

        self.configInts = ['try_generation_limit', 'knots', 'num_splines', 'num_good_splines']  # don't forget to include this one in config lists
        self.configLists = ["configInts", "configLists", "configBool", 'window',
                            'configDicts']  # don't forget to include this one in config lists
        self.configBools = ['LOGFLAG', 'splineBankOn']
        self.configDicts = []


class vehicleControlConfig(baseClass):

    # def __init__(self, maxSpeed: float, velocityFactor : float, maxAngle : float,
    #             velocityProportion : float, velocityIntegral : float, velocityDerivative : float,
    #             steeringProportion : float, steeringIntegral : float, steeringDerivative : float,
    #             brakingProportion : float, brakingIntegral : float, brakingDerivative : float,
    #             configInts : List, configLists : List):
    def __init__(self):
        self.LOGFLAG = GLOBAL_LOGFLAG or False
        self.airSimMode = True
        self.PIDSteeringON = True
        self.PIDThrottleON = True
        self.initialHeading = INITIAL_HEADING

        self.maxSpeed = MAX_SPEED
        self.coastingSpeed = 4.0
        self.minSpeed = .05
        self.baseMag = 4.0
        self.calMag = True
        self.velocityFactor = 2.0

        self.throttleProportion = .5
        self.throttleIntegral = .0015
        self.throttleDerivative = 0.4
        self.throttleMin = -1 #since need to later transform for braking
        self.throttleMax = 1

        self.throttleReverseProportion = .5
        self.throttleReverseIntegral = .0015
        self.throttleReverseDerivative = 0.38
        self.throttleReverseMin = -1 #since need to later transform for braking
        self.throttleReverseMax = 1

        self.steeringProportion = 1.6
        self.steeringIntegral = .001
        self.steeringDerivative = 0.58
        self.steeringMin = -1
        self.steeringMax = 1

        self.skipReverse = 5

        self.stuckDetection = True
        self.reverseDrive = True

        self.maxReverseSteeringAndBrakingAngle = 1.831 #pi - 3 times max angle, could also make so that there are 50 degrees left behind
        self.stuckTimeCheck = 1 #how often updates if stuck or not
        self.checkAmount = 2 #how many locations and velocities need to be checked
        self.locationDisplacementMin = .1 #the minimum location displacment meaning the car is stuck

        #self.velocityAmount = 3 #how many past velocities need to be checked
        self.velocityChangeMin = .05 #the minimum velocity change meaning the car is stuck

        self.defaultAngle = 0.4363  # math.pi/2 depends on velocity? steering curve:
        self.maxAngle = 0.4363  # math.pi/2 depends on velocity? steering curve:
        self.steeringCurveVel = [0.0, 16.2334, 43.572559, 118.64]
        self.steeringCurveRatio = [1.0, 0.605469, 0.277344, 0.2]

        self.configInts = []  # don't forget to include this one in config lists
        self.configLists = ["configInts", "configLists", "configBool",
                            'configDicts']  # don't forget to include this one in config lists
        self.configBools = ['LOGFLAG', 'reverseSteering', 'airSimMode', 'PIDSteeringON', 'PIDThrottleON']
        self.configDicts = []

    def maxAngleUpdate(self, velocity):             ###needs to be changed per vehicle
        """updates maxAngle using steering curve"""

        def read_curve(velocity):
            return np.interp(velocity, self.steeringCurveVel, self.steeringCurveRatio, left=1.0, right=0.2)

        self.maxAngle = self.defaultAngle * read_curve(velocity)
        # print("updated maxAngle: " + str(self.maxAngle))
        self.brakeAngle = self.maxAngle * 2.0


class parseOdomConfig(baseClass):

    def __init__(self):
        self.LOGFLAG = GLOBAL_LOGFLAG or False

