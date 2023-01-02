# this is a file that acts like ROS in that it facilities connecting all the files together in Motion Planning
# this is not a pipeline though, only the connections between the pipes

from controlSystems import waypointManager
from controlSystems import potentialFieldMap
from controlSystems import vehicleControl
import MainConfigClass


class miniRos(object):

    def __init__(self, carState, waypoints, edges):

        # set up all nodes/files
        self.potentialField = potentialFieldMap.potentialFieldMapNode()
        self.potentialField.config = MainConfigClass.potentialFieldConfig()

        self.vehicleControls = vehicleControl.vehicleControlNode()
        self.vehicleControls.config = MainConfigClass.vehicleControlConfig()

        self.waypointControl = waypointManager.goalOverseerNode(waypoints, edges, MainConfigClass.GOALS)
        self.waypointControl.config = MainConfigClass.goalOverseerConfig()

        # what will get the output of ROS mapped to
        self.carState = carState

        # messages
        self.directionVector = []
        self.currentGoal = []
        self.currentVelocity = 0
        self.obstacles = []
        self.currentLocation = []
        self.rayVector = []
        self.currentHeading = []

    def updateROSVars(self, obstacles, rayVector, currentLocation, currentHeading):
        self.currentHeading = currentHeading
        self.currentLocation = currentLocation
        self.obstacles = obstacles
        self.rayVector = rayVector

    def updateROS(self):
        self.updateWaypointManager()
        self.updatePotentialField()
        self.updateVehicleControls()

    def updateWaypointManager(self):
        """updates the waypoint manager and sets the current goal"""

        # update the current location and current goal
        self.waypointControl.location = self.currentLocation
        self.waypointControl.findNextWaypoint_mainHelper()

        # get the current goal
        self.currentGoal = self.waypointControl.currentGoal

    def updatePotentialField(self):
        """updates the potential field and sets the direction vector"""

        # set up potentialField
        self.potentialField.currentGoal = self.currentGoal
        self.potentialField.currentLocation = self.currentLocation  # since in local reference to vehicle
        self.potentialField.currentHeading = self.currentHeading
        if self.potentialField.config.vision:
            self.potentialField.obstacles = self.obstacles
        else:
            self.potentialField.cloudPoints = self.obstacles

        # get the direction vector
        self.directionVector = self.potentialField.directionUpdateHelper(self.rayVector)

    def updateVehicleControls(self):
        """updates vehicle controls and sets the new controls"""

        # set up vehicleControls
        self.vehicleControls.currentVelocity = self.carState.velocity
        self.vehicleControls.directionVector = self.directionVector
        self.vehicleControls.currentThrottle = self.carState.throttle
        self.vehicleControls.currentSteering = self.carState.steering
        self.vehicleControls.currentBrake = self.carState.brake
        self.vehicleControls.currentGear = self.carState.gear

        # get the new controls
        self.vehicleControls.sendControls_callbackHelper()

        self.carState.throttle = self.vehicleControls.currentThrottle
        self.carState.steering = self.vehicleControls.currentSteering
        self.carState.brake = self.vehicleControls.currentBrake
