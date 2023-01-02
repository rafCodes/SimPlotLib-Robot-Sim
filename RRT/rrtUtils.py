from RRT import SplineUtility
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np
import random, math


class RRTNode(object):
    def __init__(self, point, parent):
        super(RRTNode, self).__init__()
        self.point = point
        self.parent = parent

def default_init():
    obstacle_maps = np.load('MapSet_RRT.npy')
    map_num = np.random.randint(0, len(obstacle_maps) - 1)
    # map_num = 23
    goal_region_safe = np.array([[-10.0, 30.0, 3.0]])
    map = obstacle_maps[map_num]
    print("Loading obstacle map number {}".format(map_num))
    return map, goal_region_safe

def check_angle(p0, p1, p2, max_steer_angle):
    """
    Checks if nodes in p2 are between -14 degrees and 14 degrees from p1,
    in the direction of p1 from p0.

    Inputs:
    p0: node before the two nodes the function checks the angle for [format: (x,y)]
    p1: node that p2's direction is referenced from [format: (x,y)]
    p2: node whose angle is calculated from p1 [format: [[x], [y]]]

    Outputs:
    Returns a boolean array.
    """
    angles = np.arctan2(p2[1][:] - p1[1], p2[0][:] - p1[0])
    # print(angles)
    if p0 != None:
        init_angle = np.arctan2(p1[1] - p0[1], p1[0] - p0[0])
    else:
        init_angle = 0
    # print(init_angle)
    return np.abs(angles - init_angle) <= max_steer_angle

def check_points_rect(initialPoint, goalRegion, config):
    """
    Returns points to check for straight line pathing to a rectangular goal(s)

    Inputs:
    initialPoint : point that originates straight line paths
        format : Array[x,y]
    goalRegion : Array of rectangular destination areas
        format : Array[Array[center x, center y, width, height], ...]
    config : rrtConfigClass.rrtConfig class config object

    Outputs an array of line segments defined by start and end points
        format : [[[x1, x2], [y1, y2]], [[x1, x2], [y1, y2]], ... ]
    """
    checkPointsX, checkPointsY = [], []
    if len(goalRegion.shape) == 1:
        goalRegion = np.expand_dims(goalRegion, 0)

    # Add the 4 corners of each goal region
    checkPointsX.append(goalRegion[:, 0] - goalRegion[:, 2] / 2.0)
    checkPointsY.append(goalRegion[:, 1] - goalRegion[:, 3] / 2.0)

    checkPointsX.append(goalRegion[:, 0] + goalRegion[:, 2] / 2.0)
    checkPointsY.append(goalRegion[:, 1] + goalRegion[:, 3] / 2.0)

    checkPointsX.append(goalRegion[:, 0] - goalRegion[:, 2] / 2.0)
    checkPointsY.append(goalRegion[:, 1] + goalRegion[:, 3] / 2.0)

    checkPointsX.append(goalRegion[:, 0] + goalRegion[:, 2] / 2.0)
    checkPointsY.append(goalRegion[:, 1] - goalRegion[:, 3] / 2.0)

    # Add straight (vertical) line points
    # First check to which goals exist a vertical line
    straightLineGoals = np.logical_and(goalRegion[:, 0] - goalRegion[:, 2] / 2.0 < initialPoint.point[0],
                                       goalRegion[:, 0] + goalRegion[:, 2] / 2.0 > initialPoint.point[0])
    straightLineX = np.ones(np.sum(straightLineGoals)) * initialPoint.point[0]
    straightLineY = (goalRegion[:, 1] - goalRegion[:, 3] / 2.0)[straightLineGoals]
    # print("straightLineX ", straightLineX)
    # print("straightLineY ", straightLineY)
    checkPointsX.append(straightLineX)
    checkPointsY.append(straightLineY)

    # Check steer angle constraints
    checkPointsX = np.concatenate(checkPointsX)
    checkPointsY = np.concatenate(checkPointsY)
    validPointsMask = check_angle((initialPoint.point[0], initialPoint.point[1] - 1.0), initialPoint.point,
                                  np.stack((checkPointsX, checkPointsY)), config.MAX_STEER)
    checkPointsX = checkPointsX[validPointsMask].reshape(-1, 1)
    checkPointsY = checkPointsY[validPointsMask].reshape(-1, 1)
    checkPointsX = np.stack((np.ones_like(checkPointsX) * initialPoint.point[0], checkPointsX), axis=-1).reshape(-1)
    checkPointsY = np.stack((np.ones_like(checkPointsY) * initialPoint.point[1], checkPointsY), axis=-1).reshape(-1)
    return np.stack((checkPointsX, checkPointsY))

def check_points_circle(initialPoint, goalRegion, config):
    """
    Returns points to check for straight line pathing to a circular goal(s)

    Inputs:
    initialPoint : point that originates straight line paths
        format : Array[x,y]
    goalRegion : Array of circular destination areas
        format : Array[Array[center x, center y, radius], ...]
    config : rrtConfigClass.rrtConfig class config object

    Outputs an array of line segments defined by start and end points
        format : [[[x1, x2], [y1, y2]], [[x1, x2], [y1, y2]], ... ]
    """
    checkPointsX, checkPointsY = [], []
    if len(goalRegion.shape) == 1:
        goalRegion = np.expand_dims(goalRegion, 0)

    # Generate points along diameter of circle to check for valid straight line paths
    for goal in goalRegion:
        diameterX, diameterY = line_subdivision([goal[0] - goal[2], goal[0] + goal[2]], [goal[1], goal[1]], config.NUM_CIRCLE_CHECK_POINTS)
        checkPointsX.append(diameterX)
        checkPointsY.append(diameterY)

    # Add straight line point
    # First check to which goals exist a vertical line
    straightLineGoals = np.logical_and(goalRegion[:, 0] - goalRegion[:, 2]  < initialPoint.point[0],
                                       goalRegion[:, 0] + goalRegion[:, 2]  > initialPoint.point[0])
    straightLineX = np.ones(np.sum(straightLineGoals)) * initialPoint.point[0]
    straightLineY = (goalRegion[:, 1])[straightLineGoals]
    # print("straightLineX ", straightLineX)
    # print("straightLineY ", straightLineY)
    checkPointsX.append(straightLineX)
    checkPointsY.append(straightLineY)

    # # Check steer angle constraints
    # print(checkPointsX)
    checkPointsX = np.concatenate(checkPointsX)
    checkPointsY = np.concatenate(checkPointsY)
    validPointsMask = check_angle((initialPoint.point[0], initialPoint.point[1] - 1.0), initialPoint.point,
                                  np.stack((checkPointsX, checkPointsY)), config.MAX_STEER)
    checkPointsX = checkPointsX[validPointsMask].reshape(-1, 1)
    checkPointsY = checkPointsY[validPointsMask].reshape(-1, 1)
    checkPointsX = np.stack((np.ones_like(checkPointsX) * initialPoint.point[0], checkPointsX), axis=-1).reshape(-1)
    checkPointsY = np.stack((np.ones_like(checkPointsY) * initialPoint.point[1], checkPointsY), axis=-1).reshape(-1)
    return np.stack((checkPointsX, checkPointsY))

def line_subdivision(x, y, numPoints):
    """

    Inputs:
    x : Array of x points defining the (piecewise) line to subdivide
    y : Array of y points defining the (piecewise) line to subdivide
    numPoints : number of points to subdivide the line into

    Outputs:
    x_regular : Array of x points that evenly subdivide the line
    x_regular : Array of y points that evenly subdivide the line

    """
    distance = np.cumsum(np.sqrt(np.ediff1d(x, to_begin=0) ** 2 + np.ediff1d(y, to_begin=0) ** 2))
    distance = distance / distance[-1]

    fx, fy = interp1d(distance, x), interp1d(distance, y)

    alpha = np.linspace(0, 1, numPoints)
    x_regular, y_regular = fx(alpha), fy(alpha)
    return x_regular, y_regular

def straight_line_collision(linePoints, obstacles, sampleRate):
    """
    Inputs:
    linePoints : Array of points defining each line.
        format : [[[x1, x2], [y1, y2]], [[x1, x2], [y1, y2]], ... ]

    Outputs
    lineCollisionMask : boolean list indicating whether the line collides with an obstacle
        format : [bool,...]
    """
    linePointsX, linePointsY = linePoints
    linePointsX = linePointsX.reshape(-1, 2)
    linePointsY = linePointsY.reshape(-1, 2)


    lineCollisionMask = []
    for line in zip(linePointsX, linePointsY):
        lineX, lineY = line_subdivision(*line, sampleRate)
        noCollideMask = SplineUtility.not_in_obs_array(obstacles, lineX, lineY)
        lineCollisionMask.append(np.all(noCollideMask))
    return lineCollisionMask, np.array(list(zip(linePointsX, linePointsY)))

def dist(p1, p2):
    """
    Returns the squared distance between two points.

    Inputs:
    p1: initial point to calculate the distance from. [format: Array[x,y]]
    p2: point to calculate the distance to. [format: Array[x,y]]

    Outputs:
    Returns the distance between two points. [int]
    """
    return np.dot(p1 - p2, p1 - p2)

def step_from_to(p1,p2,delta):
    """
    Chooses the next node in the direction of p2 from p1.

    If the distance between p1 and p2 is less than delta, then we choose p2 as the next node.
    If the distance between p1 and p2 is less than delta, then we choose the nearest node that is
    in the direction of p2 from p1 of distance delta.

    Inputs
    p1: random node that we are calculating the direction from. [format: (x,y)]
    p2: new node we are calculating the direction to. [format: (x,y)]

    Outputs:
    The node in the direction of p2 from p1.
    """
    if dist(p1, p2) < delta ** 2.:
        return p2
    else:
        theta = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
        return np.array([p1[0] + delta * np.cos(theta), p1[1] + delta * np.sin(theta)])

def l2_distance(x, y):
    """
    Calculate the distance between two points.

    Inputs
    x: array of x values of the two points
    y: array pf y values of the two points

    """
    return ((x[1] - x[0]) ** 2. + (y[1] - y[0]) ** 2.) ** 0.5


# initPoint = RRTNode([0,0], None)
# circleGoals = np.asarray([[0, 30, 2]])
# rectGoals = np.asarray([[0, 30, 5, 5], [-2, 25, 5, 5], [3, 25, 5, 5], [4, 35, 9, 9]])
# import rrtConfigClass
# config = rrtConfigClass.rrtConfig()
# print(check_points_circle(initPoint, circleGoals, config))
# check_points_rect(initPoint, rectGoals, np.pi / 4.0)