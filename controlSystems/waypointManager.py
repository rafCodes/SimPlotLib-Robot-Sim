#!/usr/bin/env python
import MainConfigClass
from controlSystems import routePlanner
from controlSystems import waypointPathEditor
import toolbox
from optimization import optimizer

class goalOverseerNode(object):
    """Controls which map nodes are being targeted for motion planning"""

    def __init__(self, waypoints = {}, edges = [], goals = [],):
        self.config = MainConfigClass.goalOverseerConfig()
        self.carConfig = MainConfigClass.carConfig()
        #self.lateralPlanner = hybridSplineRouter.lateralPathGenerator()
        self.LOGFLAG = self.config.LOGFLAG
        self.heading_angle = self.config.initialHeading
        self.location = self.config.startLocation
        if waypoints == {}:
            self.waypoints = list(MainConfigClass.WAYPOINTS.values())
            self.graph, _ = routePlanner.getGraph(MainConfigClass.WAYPOINTS, MainConfigClass.EDGES)
        else:
            self.waypoints = list(waypoints.values())
            self.graph, _ = routePlanner.getGraph(waypoints, edges)
        #TODO change this! Need to load directly from MasterClass

        if goals != []:
            self.goals = goals
        else:
            self.goals = self.config.goals

        self.currentPath = waypointPathEditor.waypoint(None, routePlanner.RoadNode(0.0, 0.0), None)
        self.currentPathTail = waypointPathEditor.waypoint(None, routePlanner.RoadNode(0.0, 0.0), None)

        self.obstacles = []
        self.currentVelocity = 0

        #for optimizer
        self.innerData = []
        self.outerData = []
        self.optimalGraph = None
        self.kdTrEEGraph = None

        #no goals
        if self.goals == []:
            # tells car to not move
            self.currentGoal = []
            self.targetGoal = []

        else:
            # get goals and route to them
            self.targetGoal = self.goals.pop(0)
            if self.config.optimalPath:
                self.currentGoal = []
            else:
                self.getPath() #also gets current goal

        self.currentCrumb = 0
        if self.config.breadcrumbMode:
            self.breadCrumbs = MainConfigClass.BREADCRUMB
            self.currentGoal = self.breadCrumbs[self.currentCrumb]
        else:
            self.breadCrumbs = []

    def addNodes(self, nodes, edges):
        # removing duplicates
        newEdges = []
        for edge in edges:
            if not (edge in newEdges or [edge[1], edge[0]] in newEdges):
                newEdges.append(edge)
        edges = newEdges
        # adding edges
        for edge in edges:
            node1 = nodes[edge[0]]
            node2 = nodes[edge[1]]
            rn1 = routePlanner.RoadNode(node1[0], node1[1])
            rn2 = routePlanner.RoadNode(node2[0], node2[1])
            found1 = False
            found2 = False
            length = len(self.waypoints)
            i = 0
            # checking if already in graph
            while not (found1 and found2) and i < length:
                if not found1 and node1 == self.waypoints[i]:
                    rn1 = self.graph[i]
                    found1 = True
                if not found2 and node2 == self.waypoints[i]:
                    rn2 = self.graph[i]
                    found2 = True
                i += 1
            # adding to graph
            temp = routePlanner.Edge(routePlanner.node_euclidean(rn1, rn2), rn1, rn2)
            if not found1:
                self.graph.append(rn1)
            if not found2:
                self.graph.append(rn2)

    def getOptimalPath(self):
        """make the optimal path for the car to follow - note, does it for current targetGoal"""
        if self.optimalGraph is None:
            self.optimalGraph, self.kdTrEEGraph = optimizer.createTrackGraphAndTree(self.innerData, self.outerData,
                                                                             self.carConfig.clockSide, self.carConfig.mass,
                                                                             self.carConfig.gravity, self.carConfig.dragC,
                                                                             self.carConfig.rollingC, self.carConfig.avgVelocity)
        # list of RoadNodes
        path = None
        while path is None and self.targetGoal != []:
            #path is none when fails plan_route
            #targetGoal is [] when no more goals to try, different from no goals left since has goals can be empty
            #but the last goal popped from it may have not been tried
            path, speeds = optimizer.optimumDrive(self.kdTrEEGraph, self.location, self.targetGoal, self.carConfig.mass,
                                                  self.carConfig.gravity, self.carConfig.dragC, self.carConfig.rollingC,
                                                  self.carConfig.timeLimit, self.carConfig.timeLimitWeight)
            if self.pathNoneOrFound(path, speeds):
                break


    def getPath(self):
        """make the path for the car to follow - note, does it for current targetGoal"""
        # list of RoadNodes
        path = None
        while path is None and self.targetGoal != []:
            #path is none when fails plan_route
            #targetGoal is [] when no more goals to try, different from no goals left since has goals can be empty
            #but the last goal popped from it may have not been tried
            path = routePlanner.plan_route(self.graph, self.location, self.targetGoal, self.heading_angle)

            if self.pathNoneOrFound(path):
                break

    def pathNoneOrFound(self, path, speeds = []):
        """determines what to do with the path"""
        # no path found, don't move
        if path is None:
            # no path found, don't move
            self.currentPath = None
            self.currentGoal = []
            if len(self.goals) == 0:
                self.targetGoal = []
                # cannot get any more, failed operation
                return True #gets it out of loop
            else:
                self.targetGoal = self.goals.pop(0)
        else:
            if speeds == []:
                self.currentPath, self.currentPathTail = waypointPathEditor.buildPath(path)
                self.currentGoal = [self.currentPath.point.x, self.currentPath.point.y]
            else:
                self.currentPath, self.currentPathTail = waypointPathEditor.buildOptimalPath(path, speeds)
                self.currentGoal = [self.currentPath.point.x, self.currentPath.point.y, self.currentPath.velocity]
            return True
        return False

    def goalCheck(self, currentLocation, currentGoal, size):
        """return true if the car is close enough to the goal to determine that it has been visited: list, list"""
        goalDistance = toolbox.fastNpLinAlg([currentLocation[0] - currentGoal[0], currentLocation[1] - currentGoal[1]])
        if goalDistance <= size:
            return True
        return False

    def breadCrumbWalker(self):
        if self.currentGoal == []:
            return
        if self.goalCheck(self.location, self.breadCrumbs[self.currentCrumb], self.config.goalRadius if self.currentGoal == self.targetGoal else self.config.waypointRadius):
            self.currentCrumb += 1

            if self.currentCrumb >= len(self.breadCrumbs):
                self.currentGoal = []
                return
            else:
                self.currentGoal = self.breadCrumbs[self.currentCrumb]
                # print(self.currentGoal)


    def findNextWaypoint_mainHelper(self): #formerly goalControl
        """updates and publishes the current goal, without ROS functionality"""
        #check if any other goals inadvertently passed when doing shortest path
        if self.config.breadcrumbMode:
            self.breadCrumbWalker()
            return

        if self.config.checkIfFoundNonTargetGoal:
            count = 0
            for goal in self.goals: #TODO matrix optimization?
                if self.goalCheck(self.location, goal, self.config.goalRadius):
                    self.goals.pop(count)
                else: #fixes issue of losing track of goal order if more than one popped
                    count += 1

        if self.currentGoal == []:
            #keep sending stop signal, don't change current goal
            pass

        elif self.goalCheck(self.location, self.currentGoal, self.config.goalRadius if self.currentGoal == self.targetGoal else self.config.waypointRadius):
            # went through a goal or a waypoint
            if self.currentPath.tail is None:
                # passed though all waypoints for a path, need to determine if done
                if self.goals != []:
                    # go to the next goal
                    self.targetGoal = self.goals.pop(0)
                    self.getPath()
                else:
                    # all goals found, can stop
                    self.currentGoal = []
                    self.targetGoal = []

            else:
                # iterate to the next waypoint on the path
                #iterate linked list
                self.currentPath = self.currentPath.tail
                if self.config.optimalPath:
                    self.currentGoal = [self.currentPath.point.x, self.currentPath.point.y, self.currentPath.velocity]
                else:
                    self.currentGoal = [self.currentPath.point.x, self.currentPath.point.y]
                # send out the signal for the next waypoint

        else:
            #keep sending same signal, might not be needed since if elif does not run, state does not change
            pass
