#!/usr/bin/env python
import math
import numpy as np
import time

import MainConfigClass
import toolbox

class vehicleControlNode(object):

    def __init__(self):
        self.config = MainConfigClass.vehicleControlConfig()
        self.calcConfig = MainConfigClass.potentialFieldConfig()

        self.LOGFLAG = self.config.LOGFLAG

        self.currentVelocity = 0
        self.directionVector = [0, 0]
        self.directionVectorAngle = 0
        self.directionVectorMagnitude = 0
        self.collisionCount = 0
        self.ROSTime = 0

        # intialized in stopped configuration

        self.currentThrottle = 0
        self.currentSteering = 0
        self.currentBrake = 1
        self.currentGear = MainConfigClass.FORWARD
        #can also be "reverse"


        self.lastTime = 0
        self.lastPosition = []
        self.lastVelocity = []
        self.currentLocation = []
        self.cycleCount = 0

        #TODO no currently using but can get from ROS if it helps with checkPosition
        self.hittingSomething = False

        # PIDs
        #throttle
        self.throttlePID = toolbox.PID_template(self.config.throttleProportion, self.config.throttleIntegral,
                                                self.config.throttleDerivative, self.config.throttleMin,
                                                self.config.throttleMax)

        #throttle
        self.throttleReversePID = toolbox.PID_template(self.config.throttleReverseProportion, self.config.throttleReverseIntegral,
                                                self.config.throttleReverseDerivative, self.config.throttleReverseMin,
                                                self.config.throttleReverseMax)

        #steering
        self.steeringPID = toolbox.PID_template(self.config.steeringProportion, self.config.steeringIntegral,
                                                self.config.steeringDerivative, self.config.steeringMin,
                                                self.config.steeringMax)

    def getSteering(self, directionVector):
        # def getSteering(self, directionVector : List):
        """return the turn needed"""

        # for tuning: https://en.wikipedia.org/wiki/PID_controller#Ziegler%E2%80%93Nichols_method, http://www.cds.caltech.edu/~murray/books/AM08/pdf/am06-pid_16Sep06.pdf

        if directionVector == [] or directionVector == [0, 0]:
            return 0 #make steering go to center

        calculatedAngle = self.directionVectorAngle * toolbox.getSide([0, 1], self.directionVector)

        #Check if need reverse angle

        if self.currentGear == MainConfigClass.REVERSE:
            if abs(calculatedAngle) > self.config.maxReverseSteeringAndBrakingAngle:
                directionVector[1] *= -1
                #need to flip the steering vector, -1 used to flip direction angle so that it is point forwards
                directionVector, calculatedAngle, _ = self.getUnitDirectionVectorAndAngleNoSide(directionVector, True)
                calculatedAngle *= toolbox.getSide([0, 1], self.directionVector)

            else:
                #flipping over y axis since want to drive in direction away from goal when reversing to not be stuck again
                calculatedAngle *= -1

        if self.config.airSimMode:
            #angle is flipped for airsim
            calculatedAngle *= -1

        if self.config.PIDSteeringON:
            #make sure not over max
            absCalculatedAngle = abs(calculatedAngle)
            if absCalculatedAngle > self.config.maxAngle:
                calculatedAngle /= absCalculatedAngle

            targetSteering = self.steeringPID.update(calculatedAngle, self.currentSteering*self.config.maxAngle)

        else:
            #this is a very meh way to do it, use PID though
            targetSteering = calculatedAngle / self.config.maxAngle

            # make sure within bounds, auto done with PID
            absTargetSteering = abs(targetSteering)
            if absTargetSteering > 1: #this is a bad hardcoded number be wary of this
                targetSteering /= absTargetSteering

        return targetSteering

    def getThrottle(self, directionVector, currentVelocity):
        # def getThrottle(self, directionVector: List, currentVelocity: float):
        """return the new throttle and calculated brake"""

        # need to stop
        if directionVector == []:
            return 0, 1, 0
        # TODO: Roll to stop when goals completed

        if directionVector == [0, 0]:
            return self.currentThrottle, self.currentBrake, -1

        calculatedMag = self.directionVectorMagnitude
        calculatedAngle = self.directionVectorAngle

        if not self.config.calMag:
            calculatedMag = self.config.baseMag

        # check if over max speed, mph 20, 8.94 m/s
        if calculatedMag > self.config.maxSpeed:
           calculatedMag = self.config.maxSpeed

        # check if going backwards or if need to brake because of a obstacle, done after max speed check to reduce checking by one
        if self.currentGear != MainConfigClass.REVERSE and calculatedAngle > self.config.maxReverseSteeringAndBrakingAngle:
            calculatedMag *= -1

        #calculation stuff
        if self.config.PIDThrottleON:
            if self.currentGear == MainConfigClass.FORWARD:
                targetThrottle = self.throttlePID.update(calculatedMag, currentVelocity)
            else:
                targetThrottle = self.throttleReversePID.update(calculatedMag, currentVelocity)
        else:
            #this is not a good way to do it, use PIDs
            # TODO: Decide if we want coasting when PID is on
            # coasting functionality, helps reduce throttle, brake cycle
            if calculatedMag > self.config.coastingSpeed and currentVelocity > self.config.coastingSpeed:
                return 0.0, 0.0  # no brake, no throttle, allow car to coast

            targetThrottle = calculatedMag/self.config.maxSpeed
            if calculatedMag > 0 and calculatedMag - currentVelocity < 0:
                targetThrottle *= -1
            absTargetThrottle = abs(targetThrottle)
            if absTargetThrottle > 1: #yes this is hard coded in because this way is bad and min and max are -1 and 1
                targetThrottle /= absTargetThrottle

        #set up brake if needed
        calculatedBrake = 0.0
        if targetThrottle < 0:
            targetThrottle *= -1
            if self.currentGear != MainConfigClass.REVERSE:
                calculatedBrake = targetThrottle
                targetThrottle = 0

        return targetThrottle, calculatedBrake, calculatedMag

    def checkPosition(self, clockTime = -1):
        """check if the car is stuck"""

        stuckPosition = False
        stuckVelocity = False
        stuck = False

        if self.directionVector == [] or self.directionVectorAngle == [0 ,0]:
            #at end of track, no need to reverse or even check it
            self.currentGear = MainConfigClass.FORWARD
            return False

        if clockTime == -1:
            currentTime = time.time()
        else:
            currentTime = clockTime
        if currentTime - self.lastTime > self.config.stuckTimeCheck:
            #print('check')
            #next check cycle
            self.lastTime = currentTime
            self.lastPosition.append(self.currentLocation)
            self.lastVelocity.append(self.currentVelocity)

            if len(self.lastPosition) >= self.config.checkAmount:
                if self.cycleCount < self.config.skipReverse:
                    # print('First!')
                    # print(clockTime)
                    #to make sure does not reverse when starting
                    self.cycleCount += 1
                    self.lastPosition = []
                    self.lastVelocity = []
                    return False

                #now need to check if out of bounds based from 0th place in lastPosition
                for i in range(1, len(self.lastPosition)):
                    checkDist = [self.lastPosition[i][0]-self.lastPosition[i-1][0], self.lastPosition[i][1]-self.lastPosition[i-1][1]]
                    if toolbox.fastNpLinAlg(checkDist) <= self.config.locationDisplacementMin:
                        stuckPosition = True
                        #print('stuck position')
                        break

                for i in range(1, len(self.lastVelocity)): #assumes lastVelocity len is more than 0
                    checkVel = abs(self.lastVelocity[i] - self.lastVelocity[i-1])
                    if checkVel < self.config.velocityChangeMin and self.lastVelocity[i] < self.config.minSpeed:
                        stuckVelocity = True
                        #print('stuck velocity')
                        break

                stuck = stuckPosition and stuckVelocity

                if stuck:
                    if self.currentGear == MainConfigClass.REVERSE:
                        self.currentGear = MainConfigClass.FORWARD
                    else:
                        self.currentGear = MainConfigClass.REVERSE
                    # print('****************')
                    # print(currentTime)
                    # print('Last Positions:', str(self.lastPosition))
                    # print('Last Velocities:', str(self.lastVelocity))
                    # print('****************')
                else:
                    pass
                    if not self.config.reverseDrive:
                        self.currentGear = MainConfigClass.FORWARD
                    #otherwise, reverseDrive takes care of it
                    # message = 'Stuck Position: ' + str(stuckPosition) + ' Stuck Velocity: ' + str(stuckVelocity) + ' Clock Time ' + str(clockTime)
                    # toolbox.warning(message)
                    #input('Press Enter to continue')

                self.lastPosition = []
                self.lastVelocity = []

            return stuck
        return stuck

        #TODO
        #no movement for x seconds, or very minimal means stuck, must also mean
        #velcoity is constant, not good since could mean at constant velocity
        #can also check if are hitting something using that var
        pass

    def checkForReverse(self):
        if self.directionVectorAngle > self.config.maxReverseSteeringAndBrakingAngle:
            self.currentGear = MainConfigClass.REVERSE
        else:
            self.currentGear = MainConfigClass.FORWARD

    def getUnitDirectionVectorAndAngleNoSide(self, directionVector, onlyAngle = False):
        """DOES NOT CHECK IF [0, 0] OR [] gets direction vector, calculated angle, and magnitude of vector"""

        if not onlyAngle:
            factor = toolbox.fastNpLinAlg(directionVector)
            directionVector[0] /= factor
            directionVector[1] /= factor
        else:
            #for when already a unit vector
            factor = 1

        calculatedAngle = math.acos((np.dot(directionVector, [0, 1])))
        #calculatedAngle *= toolbox.getSide([0, 1], directionVector)

        return directionVector, calculatedAngle, factor

    def sendControls_callbackHelper(self):
        """calculate and send the controls to the vehicle helper"""
        if self.directionVector != [] and self.directionVector != [0, 0]:
            self.directionVector, self.directionVectorAngle, self.directionVectorMagnitude = self.getUnitDirectionVectorAndAngleNoSide(self.directionVector)

            if self.config.reverseDrive:
                self.checkForReverse()

        if self.config.stuckDetection:
            stuck = self.checkPosition()
        else:
            stuck = False

        self.currentThrottle, self.currentBrake, calculatedMag = self.getThrottle(self.directionVector, self.currentVelocity)
        self.currentSteering = self.getSteering(self.directionVector)
        return calculatedMag, stuck
