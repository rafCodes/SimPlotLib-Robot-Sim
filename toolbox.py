#!/usr/bin/env python
import shutil
import numpy as np
import math
import os

###
import random
import time
###

def fastNpLinAlg(vector):
    return (vector[0]*vector[0]+vector[1]*vector[1])**.5

def pathCheck(fileFolderName, path):
# def pathCheck(fileFolderName: str, path: str):
    """format the fileFolderPath correctly for generatre folder and gernerate path"""

    rootInFolderName = './' in fileFolderName

    if rootInFolderName and path == './':
        fileFolderPath = fileFolderName
    elif path == './' and rootInFolderName:
        index = fileFolderName.find('./')
        fileFolderName = fileFolderName.replace('./', '')
        fileFolderPath = path + fileFolderName
    elif path == './':
        fileFolderPath = path + fileFolderName
    else:
        fileFolderPath = path + '/' + fileFolderName

    return fileFolderPath


def generateFolder(folderName, checkDuplicate = True, path = './'):
# def generateFolder(folderName: str, checkDuplicate: bool = True, path: str = './'):
    """generates folder needed and returns path to folder, path needs ./ infront
    if checkDuplicate is True, will make a folder if duplicate, otherwise does not"""

    folderPath = pathCheck(folderName, path)

    # checking for duplicates
    count = 0  # so you can have same folder name but a different version
    if checkDuplicate:
        while os.path.isdir(folderPath + '_' + str(count)):
            count += 1

        folderPath = folderPath + '_' + str(count)

    try:
        os.mkdir(folderPath)
        print('Folder for', folderName, 'created at path', folderPath)
    except:
        print('Folder for', folderName, 'already exists')

    print()

    return folderPath


def warning(message):
# def warning(message: str):
    """nice little warning message for when things go wrong"""
    print('*****Warning*****')
    print(message)
    print('*****************')

def pathDistance(points):
    """returns total distance in path of consecutive coordinates in form: [[x1, y1], [x2, y2], ...]"""
    total = 0
    for i in range(len(points)-1):
        total += ((points[i+1][0] - points[i][0])**2 + (points[i+1][1] - points[i][1])**2)**.5
    return total

def generateFile(path, fileName, ending):
# def generateFile(path: str, fileName: str, ending: str):
    """generates file inside path, ending is file ending like .txt, returns filePath"""

    filePath = pathCheck(fileName, path)

    # checking for duplicates
    count = 0  # so you can have same folder name but a different version
    while os.path.isfile(filePath + '_' + str(count) + ending):
        count += 1
    filePath = filePath + '_' + str(count) + ending
    file = open(filePath, 'w+')
    file.close()

    print('File for', fileName, 'created at path', filePath)

    return filePath

def compareTwoValues(value1, value2, tolerance):
# def compareTwoValues(value1: float, value2: float, tolerance: float):
    """compare two values with a tolerance, tolerance is max difference between the values"""

    # check if same sign
    if value1 < 0:
        assert (value2 <= 0)
    elif value1 > 0:
        assert (value2 >= 0)

    difference = abs(abs(value1) - abs(value2))
    assert (difference < tolerance)


def compareClassValues(class1, class2):
    """compares all the values in each class to see if they are identical"""
    assert (class1.__class__.__name__ == class2.__class__.__name__)
    class1Vars = class1.__dict__
    class2Vars = class2.__dict__

    assert (len(class1Vars) == len(class2Vars))

    # test values in the class
    for key in class1Vars:
        item1 = class1Vars[key]
        item2 = class2Vars[key]
        #print(key, hasattr(item1, '__dict__'))
        # checking if need to check through an array
        if type(item1) == np.ndarray and type(item2) == np.ndarray:
            assert (len(item1) == len(item2))
            for index in range(0, len(item1)):
                assert (item1[index] == item2[index])

        # checking if has nested class
        elif hasattr(item1, '__dict__') and hasattr(item2, '__dict__'):
            compareClassValues(item1, item2)

        # compares basic python type
        else:
            #print("item1", item1)
            #print("item2", item2)
            assert (item1 == item2)

def rectangleFromTwoPoints(c1, c2):
    """make a rectangle from 2 points"""

    rect = []

    w = c1[0] - c2[0]
    h = c1[1] - c2[1]

    if w > 0:  # c1 on right
        x = c1[0] - abs(w) / 2
    else:      # c2 on right
        x = c2[0] - abs(w) / 2

    if h > 0:  # c1 on top
        y = c1[1] - abs(h) / 2
    else:  # c1 on bottom
        y = c2[1] - abs(h) / 2

    rect = [x, y, abs(w), abs(h)]

    return rect

def startTesting():
    generateFolder('testing', False)
    path = generateFolder('testing/temp')

    return path


def cleanUp(path):
# def cleanUp(path: str):
    """Cleans up and removes a path folder"""
    print()
    try:
        shutil.rmtree(path)
        print(path, 'cleaned up')
    except:
        ('print temp folder gen failed')
    print()


"""def circleRectangleWrongDistance(radius, circleXY, rectangle):
    ""this does not find the shortest distance between a circle and rectangle, don't use this, left here so not
    accidentally implemented again""

    #first check if circle is in rectangle
    halfWidth = rectangle[2]/2
    halfHeight = rectangle[3]/2
    if circleXY[0] < rectangle[0]+halfWidth and circleXY[0] > rectangle[0]-halfWidth:
        if circleXY[1] < rectangle[1]+halfHeight and circleXY[1] > rectangle[1]-halfHeight:
            return 0

    #find distance between rectangle and circle, not including inside distances
    xComponent = circleXY[0]-rectangle[0]
    yComponent = circleXY[1]-rectangle[1]
    distanceVector = [xComponent, yComponent]
    distanceVectorMag = np.linalg.norm(distanceVector)
    distance = distanceVectorMag

    #subtract circle distance, checked first because if is negative, is faster then to calculate rectangleDistance
    distance -= radius

    if distance < 0:
        return 0

    if rectangle[2] == 0 or rectangle[3] == 0:
        #is a 1 or 0 dimension rectangle, not acceptable to find a distance
        return distance

    #find rectangle distance

    sideVector = [0, halfWidth]

    unit_vector_1 = np.asarray(distanceVector)/ distanceVectorMag
    unit_vector_2 = np.asarray(sideVector) / halfWidth
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)
    if angle > 1.57: #pi/2
        angle = angle%1.57

    rectangleDistance = sideVector[1]/np.cos(angle)

    #subtract rectangle distance
    distance -= rectangleDistance

    if distance < 0:
        return 0

    return distance
"""


def getDistance(obstacle, point, pointWidth, pointHeight):
    # def getDistance(self, obstacle: Array, point: Array):
    """return the shortest distance between two rectangles or a rectangle and a point, returns 0 if there is overlap"""
    # overlaps in x or y:
    xChange = abs(obstacle[0] - point[0]) - float(obstacle[2] + pointWidth) / 2
    if xChange <= 0:
        dx = 0
    else:
        dx = xChange

    yChange = abs(obstacle[1] - point[1]) - float(obstacle[3] + pointHeight) / 2
    if yChange <= 0:
        dy = 0
    else:
        dy = yChange

    if obstacle[0] > point[0]:
        dx *= -1

    if obstacle[1] > point[1]:
        dy *= -1

    return [dx, dy]


def fiveNumberSummary(data, showInfo = True):
    """returns and prints the 5 number summary of a list of data"""

    #reference: https://machinelearningmastery.com/how-to-calculate-the-5-number-summary-for-your-data-in-python/
    data = np.asarray(data)
    quartiles = np.percentile(data, [25, 50, 75])
    # calculate min/max
    data_min, data_max = np.min(data), np.max(data)
    # print 5-number summary
    if showInfo:
        print('Five number summary for data')
        print('Min:', data_min)
        print('Q1:', quartiles[0])
        print('Median:', quartiles[1])
        print('Q3:', quartiles[2])
        print('Max:', data_max)

    return [data_min, quartiles[0], quartiles[1], quartiles[2], data_max]

def meanAndStd(data, showInfo = True):
    """returns and prints the mean and standard deviation of a list of data"""
    std = np.std(data)
    mean = np.mean(data)

    return mean, std

def getVectorAngle(vectorStart, vectorEnd):
    """Get the vector angle from two points as lists"""
    vector = [vectorEnd[0] - vectorStart[0], vectorEnd[1] - vectorStart[1]]
    factor = fastNpLinAlg(vector)
    if factor != 0:
        vector[0] /= factor
        vector[1] /= factor
    angle = math.atan2(vector[1], vector[0])
    #it should be in this order: https://numpy.org/doc/stable/reference/generated/numpy.arctan2.html
    if angle < 0:
        angle = 6.28318 + angle
    return angle, vector

class PID_template:

    def __init__(self, proportional, integral, derivative, minimum, maximum):
        self.integrator = 0
        self.derivator = 0
        self.previous = 0
        self.minimum = minimum
        self.maximum = maximum
        self.error = 0
        self.targetValue = 0

        # C means comes from config
        self.proportionalC = proportional
        self.integralC = integral
        self.derivativeC = derivative

    def update(self, calculated, processVariableCurrentValue):
        self.error = calculated - processVariableCurrentValue
        self.integrator += self.error
        self.derivator += self.previous - self.error
        self.previous = self.error
        self.targetValue = self.error * self.proportionalC + self.integralC*self.integrator + self.derivativeC*self.derivator

        if self.targetValue > self.maximum:
            self.targetValue = self.maximum

        elif self.targetValue < self.minimum:
            self.targetValue = self.minimum

        return self.targetValue



def getSide(anchorVector, relativeVector):
# def getSide(self, anchorVector: Array, relativeVector: Array):
    """get side of angle relativeVector relative to anchor vector, -1 for left, 1 for right"""
    # https://stackoverflow.com/questions/13221873/determining-if-one-2d-vector-is-to-the-right-or-left-of-another

    dot = anchorVector[0] * -relativeVector[1] + anchorVector[1] * relativeVector[0]

    if dot >= 0:
        return 1
    else:
        return -1