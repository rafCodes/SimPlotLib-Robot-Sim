import time
import numpy as np
import RRT.bezier as bezier
import RRT.rrtUtils as rrtUtils
import RRT.rrtConfigClass as rrtConfigClass
from RRT import SplineUtility
from RRT.rrtUtils import RRTNode

class rrtPathGenerator(object):

    def __init__(self):
        self.config = rrtConfigClass.rrtConfig()

    def generateRoute(self, searchWindow, obstacleMap, goalRegion):
        if self.config.GOAL_SHAPE == rrtConfigClass.CIRCLE:
            if goalRegion.size != 3:
                raise ValueError("Input argument goalRegion should be in the format [x, y, radius]")
        elif self.config.GOAL_SHAPE == rrtConfigClass.RECTANGLE:
            if goalRegion.size != 4:
                raise ValueError("Input argument goalRegion should be in the format [x, y, width, height]")
        else:
            raise ValueError("Invalid goalRegion shape config")

        currentState = 'init'
        iterationsRemaining = self.config.TIMES_TO_RUN
        if len(goalRegion.shape) == 1:
            goalRegion = np.expand_dims(goalRegion, 0)

        while iterationsRemaining > 0:

            if currentState == 'init':
                start_time = time.time()
                goalNode = None
                nodes = []
                bezierPoints = []  # points in the bezier curve
                bezierEdges = []  # points in pathPoints to connect in forming the bezier curve
                pathPoints = []  # nodes that make up the red line
                initialPoint = RRTNode(self.config.INITIAL_POINT, None)  # TODO: local vehicle coordinates
                nodes.append(initialPoint)

                if self.config.GOAL_SHAPE == rrtConfigClass.RECTANGLE:
                    checkPoints = rrtUtils.check_points_rect(initialPoint, goalRegion, self.config)
                elif self.config.GOAL_SHAPE == rrtConfigClass.CIRCLE:
                    checkPoints = rrtUtils.check_points_circle(initialPoint, goalRegion, self.config)
                isFreePath, linePoints = rrtUtils.straight_line_collision(checkPoints, obstacleMap, self.config.COLLISION_SAMPLE_RATE)
                if any(isFreePath):
                    currentState = 'goalFoundStraight'
                    end_time = time.time() - start_time
                    print('total time: {:.3f}'.format(end_time))
                    straightX, straightY = linePoints[isFreePath][0] #TODO Heuristic to choose one of the valid straight line paths.
                    numPoints = int(rrtUtils.l2_distance(straightX, straightY) / self.config.STEP_DELTA) # Subdivide line based on maximum rrt step length
                    return rrtUtils.line_subdivision(straightX, straightY, numPoints)
                else:
                    currentState = 'buildTree'

            elif currentState == 'goalFoundStraight':
                currentState = 'init'

            elif currentState == 'goalFound':
                currNode = goalNode.parent
                pathPoints.append(goalNode.point)
                while currNode.parent:
                    pathPoints.append(currNode.point)
                    currNode = currNode.parent

                if len(pathPoints):
                    end_time = time.time() - start_time
                    print('rrt time: {:.3f}'.format(end_time))
                    pathPoints = np.stack(pathPoints[::-1], axis=1).reshape(2,-1)
                    smoothed_path = bezier.apply_bezier_to_path(pathPoints, self.config.BEZIER_STEP)
                    smmoothed_path_array = np.asarray(smoothed_path).T if smoothed_path is not None else pathPoints
                    if all(SplineUtility.not_in_obs_array(obstacleMap, smmoothed_path_array[0], smmoothed_path_array[1])):
                        return smmoothed_path_array
                    else:
                        print("Smoothed path violates collision constraints. Returning unsmoothed path.")
                        # TODO: Better path smoothing algo
                    return pathPoints

            elif currentState == 'buildTree':
                if len(nodes) < self.config.NUMNODES:
                    foundNext = False
                    while not foundNext:
                        randX, randY = SplineUtility.sample_points_not_in_obs_array(searchWindow, obstacleMap, 1)
                        randPoint = np.array([randX[0], randY[0]])
                        parentNode = nodes[0]
                        prevNode = parentNode
                        for p in nodes:
                            if rrtUtils.dist(p.point, randPoint) <= rrtUtils.dist(prevNode.point, randPoint):
                                newPoint = rrtUtils.step_from_to(p.point, randPoint, self.config.STEP_DELTA)
                                # pivotPoints = np.concatenate((p.point.reshape(2,1), randPoint.reshape(2,1)),axis=1)
                                isFreePivot = SplineUtility.not_in_obs_array(obstacleMap, np.array([newPoint[0]]), np.array([newPoint[1]])) #TODO: replace with line collision check
                                if all(isFreePivot):
                                    foundNext = True
                                    parentNode = p
                                    pivotPoint = newPoint
                                prevNode = p

                    nodes.append(RRTNode(pivotPoint, parentNode))

                    if self.config.GOAL_SHAPE == rrtConfigClass.RECTANGLE:
                        if any(~SplineUtility.not_in_obs_array(goalRegion, np.array([newPoint[0]]), np.array([newPoint[1]]))):
                            currentState = 'goalFound'
                            goalNode = nodes[-1]
                    elif self.config.GOAL_SHAPE == rrtConfigClass.CIRCLE:
                        if any(SplineUtility.in_circle_array(goalRegion, np.array([newPoint[0]]), np.array([newPoint[1]]))):
                            currentState = 'goalFound'
                            goalNode = nodes[-1]

                else:
                    print("RETRYING RRT")
                    currentState = 'init'
                    iterationsRemaining -= 1
        # could not path
        return None