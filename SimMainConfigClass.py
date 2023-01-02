import MainConfigClass

class simulationBaseClassConfig(object):

    def __init__(self):

        # super(newClassName, self).__init__()
        self.ignoredDebug = ['config', 'display']
        self.graphSpace = [160.0, 1152.0, 158.40000000000003, 844.8]  #[81, 576, 79, 420] #xLow, xHigh, yLow, yHigh
        self.arrowHeadSize = 10
        self.arrowWidth = 1
        self.adjustBottom = 0.165  # shift graph up for controls at bottom
        self.windowSize = [50, 50, 300, 300]
        self.legend = ['X Legend','Y Legend']
        self.title = 'Base Simulation Class'

        self.shiftX = 0
        self.shiftY = 0

        self.goalRadius = MainConfigClass.GOAL_RADIUS
        self.goalBoostRadius = 1
        self.waypointRadius = 2

        self.activeClick = True
        self.activeDrag = False

        #.02 between button width is good, .01 is good between button height
        # format for button data: location [bottom left x, bottom left y, w, h,], name string, function
        # {'location':[x, y, w, h],'name':'', 'function':''}
        # #go to next map and save points button? would need to tie waypoints to each individual map
        self.buttons = {
            'toggleClick': {'location': [0.17, .01, 0.15, 0.04], 'name': 'Click Toggle', 'function': ''},
            'toggleDrag': {'location': [0.34, .01, 0.15, 0.04], 'name': 'Drag Toggle', 'function': ''},
            'randomDebug': {'location': [0.17, .06, 0.15, 0.04], 'name': 'Random', 'function': ''},
            'debug': {'location': [0.05, .01, 0.1, 0.04], 'name': 'Debug', 'function': ''},
            'reset': {'location': [0.05, .06, 0.1, 0.04], 'name': 'Reset', 'function': ''},
            'viewConverted': {'location': [0.62, .95, 0.15, 0.04], 'name': 'Convert Map', 'function': ''},
            'exportToJSON': {'location': [0.79, .95, 0.17, 0.04], 'name': 'Export to JSON', 'function': ''},
            'nextMap': {'location': [0.05, .95, 0.17, 0.04], 'name': 'Next Map', 'function': ''},
            'previousMap': {'location': [0.05, .9, 0.17, 0.04], 'name': 'Previous Map', 'function': ''}
        }

        # format for slider data: location [bottom left x, bottom left y, w, h,],
        # name string, min value, max value, function, color, initialValue, stepValue if stepValue is -1 then ignore
        # {'location':[x, y, w, h],'name':'','minValue':0, 'maxValue':0, 'initialValue':0, 'color': '', 'stepValue':-1, 'function':''}
        self.sliders = {}

        # format for adding a text box: location [bottom left x, bottom left y, w, h,], name string, initial text, function
        # {'location':[x, y, w, h],'name':'', 'function':'','initialText': ''}
        self.textBoxes = {}


class displaySimToolsConfig(object):

    def __init__(self):
        self.arrowXDefault = .1
        self.arrowYDefault = .1
        self.clickFunction = self.tempClick
        self.updateClickArea = self.tempAreaUpdate
        self.enableClicking = False
        self.enableMoving = False
        self.enableResizing = False
        self.moveCountMax = 3
        self.hidePlot = False

    def tempClick(self, button, mapX, mapY, displayX, displayY):
        """temporary function that does nothing"""
        print('Clicking not linked, need to link before initializing displaySim, initialize config first and change')

    def tempAreaUpdate(self, xWidthSmall, xWidthBig, yWidthSmall, yWidthBig):
        """temporary function that does nothing"""
        print(
            'Resizing not linked, need to link before initializing displaySim, initialize config first and change')


class SimulationUnitTestConfig(simulationBaseClassConfig):

    def __init__(self):

        super(SimulationUnitTestConfig, self).__init__()

        self.shiftX = 227
        self.shiftY = 273.3
        self.buttons = {**self.buttons,
                        **{'buttonTest': {'location': [0.30, .95, 0.17, 0.04], 'name': 'button', 'function': ''}}}

        self.sliders = {**self.sliders,
                        **{'sliderTest': {'location': [0.7, 0.01, 0.2, 0.03], 'name': 'slider', 'minValue': 0, 'maxValue': 20,
                                'initialValue': 10, 'color': 'lightgoldenrodyellow', 'stepValue': -1, 'function': ''}}}

        self.textBoxes = {**self.textBoxes,
                          **{'textBoxTest': {'location': [0.7, 0.05, 0.2, 0.04], 'name' : 'textbox ', 'function' : '','initialText' : 'textbox text'}}}

class benchmarkingDisplayConfig(displaySimToolsConfig):

    def __init__(self):
        super(benchmarkingDisplayConfig, self).__init__()
        ### list changes here
        self.hidePlot = True

class waypointGenerationDisplayConfig(displaySimToolsConfig):

    def __init__(self):
        super(waypointGenerationDisplayConfig, self).__init__()
        ### list changes here
        self.enableClicking = True
        self.enableMoving = True
        self.enableResizing = True

class carCalibrationSimDisplayConfig(displaySimToolsConfig):

    def __init__(self):
        super(carCalibrationSimDisplayConfig, self).__init__()
        self.enableResizing = True
        ### list changes here

class BMGSimDisplayConfig(displaySimToolsConfig):

    def __init__(self):
        super(BMGSimDisplayConfig, self).__init__()
        self.enableClicking = True
        self.enableMoving = True
        self.enableResizing = True
        ### list changes here

class RRTSimDisplayConfig(displaySimToolsConfig):

    def __init__(self):
        super(RRTSimDisplayConfig,self).__init__()
        self.enableClicking = True
        self.enableMoving = True

class SimulationUnitTestDisplayConfig(displaySimToolsConfig):

    def __init__(self):
        super(SimulationUnitTestDisplayConfig, self).__init__()
        ### list changes here
        self.enableClicking = True
        self.enableMoving = True
        self.enableResizing = True

class BMGSimConfig(simulationBaseClassConfig):

    def __init__(self):
        super(BMGSimConfig, self).__init__()

        self.windowSize = [0, 0, 70, 70]
        self.obstacleDenominator = 6
        self.baseObsHeight = 1
        self.baseObsWidth = 1
        self.randomWH = True
        self.title = ''
        self.legend = ['X (m)', 'Y (m)']

        self.protectedArea = [0, 0, 5, 5]

        self.buttons = {**self.buttons,
                        **{'accept': {'location': [0.30, .95, 0.1, 0.04], 'name': 'Accept', 'function': ''},
                           'reject': {'location': [0.30, .90, 0.1, 0.04], 'name': 'Reject', 'function': ''},
                           'generate': {'location': [0.42, .95, 0.12, 0.04], 'name': 'Generate', 'function': ''},
                           'last': {'location': [0.42, .90, 0.13, 0.04], 'name': 'Go to last', 'function': ''}}}

class RRTSimConfig(simulationBaseClassConfig):

    def __init__(self):
        super(RRTSimConfig,self).__init__()

        self.windowSize = [0, 17.5, 40, 45]
        self.goalRegion = [-10.0, 30.0, 3.0]
        self.title = 'RRT Simulation'
        self.legend = ['X (m)', 'Y (m)']

class waypointGenerationConfig(simulationBaseClassConfig):

    def __init__(self):
        super(waypointGenerationConfig, self).__init__()

        self.nearPoint = 15

        self.ignoredDebug += ['potentialFieldConfig', 'wM', 'vC']

        self.simWaitTime = 0
        self.showTime = False
        self.timeIndexIncrement = 1
        self.carPathRadius = 2.2
        self.waypointPathSimRadius = 5

        # format for button data: location [bottom left x, bottom left y, w, h,], name string, function
        # {'location':[x, y, w, h],'name':'', 'function':''}
        # #go to next map and save points button? would need to tie waypoints to each individual map
        self.buttons = {**self.buttons,
            **{'carPath': {'location': [0.27, .95, 0.17, 0.04], 'name': 'Show Car Path', 'function': ''},
            'driveCar': {'location': [0.27, .9, 0.17, 0.04], 'name': 'Drive Car', 'function': ''},
            'makeObstacleORMakeWaypoint': {'location': [0.46, .95, 0.14, 0.04], 'name': 'Obs/WP', 'function': ''},
            'placeStart': {'location': [0.46, .90, 0.14, 0.04], 'name': 'Place Start', 'function': ''},
            'viewConverted': {'location': [0.62, .95, 0.15, 0.04], 'name': 'Convert Map', 'function': ''},
            'simPath': {'location': [0.58, .06, 0.18, 0.04], 'name': 'Simulate Path', 'function': ''},
            'simPathIncrement': {'location': [0.794, .06, 0.18, 0.04], 'name': 'Move forward', 'function': ''},
            'simPathDecrement': {'location': [0.794, .01, 0.18, 0.04], 'name': 'Move backward', 'function': ''}}}

        #no decrement since would be going back in time, state of waypointManager cannot be undone, plus shows
        #previous location, unless would just be cycling previous locations?

        # format for slider data: location [bottom left x, bottom left y, w, h,],
        # name string, min value, max value, function, color, initialValue, stepValue if stepValue is -1 then ignore
        # {'location':[x, y, w, h],'name':'','minValue':0, 'maxValue':0, 'initialValue':0, 'color': '', 'stepValue':-1, 'function':''}
        self.sliders = {'driveCarTimeChange': {'location': [0.56, 0.01, 0.2, 0.03], 'name': 'Time', 'minValue': 1, 'maxValue': 50,
                  'initialValue': 0, 'color': 'lightgoldenrodyellow', 'stepValue': 1, 'function': ''}}

        # format for adding a text box: location [bottom left x, bottom left y, w, h,], name string, initial text, function
        # {'location':[x, y, w, h],'name':'', 'function':'','initialText': ''}
        self.textBoxes = {}

        self.windowSize = [-85, -125, 300, 300]
        #self.windowSize = [150,150,300,300]
        self.legend = ['',''] #['x[m]', 'y[m]']
        self.title = '' #'Current Map'


class carCalibrationSim(simulationBaseClassConfig):

    def __init__(self):

        super(carCalibrationSim, self).__init__()

        self.timeChange = 1
        self.showSim = False
        self.steeringRate = .01

        # vehicle numbers
        self.mass = 600  # kg
        self.maxBrake = 1500
        self.maxHandBrake = 3000
        self.height = 1.7
        self.frontWheelCenter = 3.28  # starting from back as 0
        self.backWheelCenter = .95

        self.updateVehicleControls = True

        # format for button data: location [bottom left x, bottom left y, w, h,], name string, function
        # {'location':[x, y, w, h],'name':'', 'function':''}
        self.buttons = {'changeMode': {'location': [0.70, .01, 0.17, 0.04], 'name': 'Change Mode', 'function': ''},
                        'updateVehicleControls': {'location': [0.55, .01, 0.13, 0.04], 'name': 'VCUpdate', 'function': ''}}

        # format for slider data: location [bottom left x, bottom left y, w, h,],
        # name string, min value, max value, function, color, initialValue, stepValue if stepValue is -1 then ignore
        # {'location':[x, y, w, h],'name':'','minValue':0, 'maxValue':0, 'initialValue':0, 'color': '', 'stepValue':-1, 'function':''}
        self.sliders = {'Rep': {'location': [0.15, 0.01, 0.3, 0.03], 'name': 'Rep', 'minValue': 0, 'maxValue': 20,
                                'initialValue': 0, 'color': 'lightgoldenrodyellow', 'stepValue': -1, 'function': ''},
                        'RepMult': {'location': [0.15, 0.06, 0.3, 0.03], 'name': 'RepMult', 'minValue': 0,
                                    'maxValue': 20, 'initialValue': 0, 'color': 'lightgoldenrodyellow', 'stepValue': -1,
                                    'function': ''},
                        'Attrac': {'location': [0.15, 0.11, 0.3, 0.03], 'name': 'Attrac', 'minValue': 0, 'maxValue': 20,
                                  'initialValue': 0, 'color': 'lightgoldenrodyellow', 'stepValue': -1, 'function': ''},

                        'velocityProportion': {'location': [0.62, 0.16, 0.3, 0.03], 'name': 'VProp', 'minValue': 0,
                                               'maxValue': 2, 'initialValue': 0, 'color': 'lightgoldenrodyellow',
                                               'stepValue': -1, 'function': ''},
                        'velocityIntegral': {'location': [0.62, 0.11, 0.3, 0.03], 'name': 'VIntg', 'minValue': 0,
                                             'maxValue': 2, 'initialValue': 0, 'color': 'lightgoldenrodyellow',
                                             'stepValue': -1, 'function': ''},
                        'velocityDerivative': {'location': [0.62, 0.06, 0.3, 0.03], 'name': 'VDer ', 'minValue': 0,
                                               'maxValue': 2, 'initialValue': 0, 'color': 'lightgoldenrodyellow',
                                               'stepValue': -1, 'function': ''},

                        'MaxPotential': {'location': [0.15, 0.16, 0.3, 0.03], 'name': 'MaxPotential', 'minValue': 0,
                                         'maxValue': 20, 'initialValue': 0, 'color': 'lightgoldenrodyellow',
                                         'stepValue': -1, 'function': ''}}

        # format for adding a text box: location [bottom left x, bottom left y, w, h,], name string, initial text, function
        # {'location':[x, y, w, h],'name':'', 'function':'', 'initialText': ''}
        self.textBoxes = {}

        # format for senarios 'name' : {'goal':[x,y], 'obstacles': [[x,y],[x,y],[x,y]], 'rayVector':[]}

        self.baseSenario = {'goal': [0, 10], 'obstacles': [], 'rayVector': []}

        self.senarios = {'turnLeftSharp': {'goal': [2, 16], 'obstacles': [[4, 1], [2, 4], [.5, 8]], 'rayVector': []},
                         'forward': {'goal': [1, 10], 'obstacles': [[4, 5], [-4, 5]], 'rayVector': []},
                         'turnRight': {'goal': [-10, 5], 'obstacles': [], 'rayVector': [0, 7]},
                         'forwardLeaning': {'goal': [1, 10], 'obstacles': [[4, 5], [-2, 5]], 'rayVector': []},
                         'baseSenario': self.baseSenario}

        self.adjustBottom = .25  # shift graph up for controls at bottom
        self.windowSize = [0, 0, 70, 70]
        self.legend = ['','']# ['x[m]', 'y[m]']
        self.title = 'Current Map'
        #TODO fix spelling
        self.senarioNumber = 0
        self.senarioIndex = ['turnLeftSharp','forward', 'turnRight', 'forwardLeaning', 'baseSenario']#self.senarios.keys()
        self.currentSenario = self.senarioIndex[0]
        self.obstacleWidth = 2
        self.obstacleHeight = 2

    def nextMode(self):
        """goes to the next mode"""
        self.senarioNumber += 1

        if self.senarioNumber > len(self.senarioIndex) - 1:
            self.senarioNumber = 0

        self.currentSenario = self.senarioIndex[self.senarioNumber]