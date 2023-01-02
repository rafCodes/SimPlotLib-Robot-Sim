#!/usr/bin/env python
import math


from controlSystems.RoadNodeClasses import *
import toolbox
import MainConfigClass


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

################################ Graph Classes ################################

def getGraph(waypoints, edges):
    """builds a map from the data passed in from MasterClass"""
    nodeSet = []
    edgeSet = []

    for waypoint in waypoints:
        nodeSet.append(RoadNode(waypoints[waypoint][0], waypoints[waypoint][1]))

    for edge in edges:
        point1 = nodeSet[edge[0]]
        point2 = nodeSet[edge[1]]
        edgeSet.append(Edge(node_euclidean(point1, point2), point1, point2))

    return nodeSet, edgeSet

def getGraphFromMapJSON(mapData):
    node_set = []

    tempWaypoints = {}
    for key in mapData.waypoints:
        tempWaypoints[int(key)] = mapData.waypoints[key]
    mapData.waypoints = tempWaypoints

    for waypoint in mapData.waypoints:
        node_set.append(RoadNode(mapData.waypoints[waypoint][0], mapData.waypoints[waypoint][1]))

    edge_set = []

    for edge in mapData.edges:
        point1 = node_set[edge[0]]
        point2 = node_set[edge[1]]
        edge_set.append(Edge(node_euclidean(point1, point2), point1, point2))
    return node_set, edge_set, mapData.obstacles

def plot_graph(edges):
    import matplotlib.pyplot as plt
    x = []
    y = []
    for edge in edges:
        # x.append(node.x)
        # y.append(node.y)
        plt.plot([edge.nodes[0].x, edge.nodes[1].x], [edge.nodes[0].y, edge.nodes[1].y], '-')
    plt.show()

def node_euclidean(node1, node2):
    return toolbox.fastNpLinAlg([node1.x-node2.x, node1.y-node2.y])

class Node:
    """
    Class representing a graph node
    """

    def __init__(self):
        self.edges = []     # Contains edges containing this node
        self.neighbors = [] # Contains neighboring nodes

    def _add_edge(self, edge):
        """ Add an edge to this Node """
        self.edges.append(edge)
        for node in edge.nodes:
            if node != self and node not in self.neighbors:
                self.neighbors.append(node)

    def _add_edges(self, edges):
        """ Add multiple edges to this Node """
        self.edges.extend(edges)
        for edge in edges:
            for node in edge.nodes:
                if node != self and node not in self.neighbors:
                    self.neighbors.append(node)

    def find_edges_to(self, other_node):
        """ Returns a list of edges connecting this node and [other_node] """
        return filter(lambda n: other_node in n.nodes, self.edges)


class Edge:
    """
    Class representing an undirected weighted edge
    """

    def __init__(self, weight, node1, node2):
        assert node1 != node2
        self.weight = weight
        self.nodes = (node1, node2)
        node1._add_edge(self)
        node2._add_edge(self)

    def getOther(self,node):
        """
        Return the other node of the edge.
        Parameter node: Node where this edge 'begins'
        Precondition: This edge is connected to this Node.
        """
        assert node in self.nodes
        if node==self.nodes[0]:
            return self.nodes[1]
        return self.nodes[0]


class RoadNode(Node):
    """
    Class representing a road feature as a Node with an Airsim coordinate
    """

    def __init__(self, x, y):
        Node.__init__(self)
        self.x = x
        self.y = y

    def __str__(self):
        return str((self.x, self.y))
    
    def removeEdge(self, node):
        """remove an edge from this node and the opposing node"""
        if node not in self.neighbors:
            print('Node not in neighbors')
            return

        for i in range(0, len(self.edges)):
            node2 = self.edges[i].nodes[0]
            if node == node2:
                self.edges.pop(i)
                self.neighbors.remove(node)
                break
            node2 = self.edges[i].nodes[1]
            if node == node2:
                self.edges.pop(i)
                self.neighbors.remove(node)
                break

        for i in range(0, len(node.edges)):
            if node.edges[i].nodes[0] == self or node.edges[i].nodes[1] == self:
                node.edges.pop(i)
                break
        node.neighbors.remove(self)

########################### Shortest Path Finder ##############################


def plan_route(graph, current_position, goal_point, heading=2*math.pi): # type (List[RoadNode], Tuple[float, float], Tuple[float, float], float between 0 and 2pi
    """
    Return an ordered list of nodes representing the shortest? path starting from the node closest
    to [current_position] to the node closest to [goal_point].
    """
    #never have a node with 1 edge
    #TODO add a set of reachable nodes that have at least 1 edge

    graphKDTree = RoadNodeKDTree(graph)
    start = graphKDTree.getClosestNode(current_position)

    badNodes = []

    if not MainConfigClass.IGNORE_REVERSE:
        vectorStart = [start.x, start.y]
        for neighbor in start.neighbors:
            vectorEnd = [neighbor.x, neighbor.y]
            angle, vector = toolbox.getVectorAngle(vectorStart, vectorEnd)
            #heading is from 0 to 2pi, 2nd is since could have 6.1 and 0 as angles but they are nearly in the same direction
            if abs(heading-angle) > MainConfigClass.ANGLE_DIFFERENCE and abs(heading - angle) < MainConfigClass.DOUBLE_PI_ANGLE_DIFFERENCE:
                badNodes.append(neighbor)
                start.removeEdge(neighbor)
                graphKDTree = RoadNodeKDTree(graph)

        endFound = False
        while not endFound:
            end = graphKDTree.getClosestNode(goal_point)
            if goal_point[0] != end.x and goal_point[1] != end.y:
                #broke, add one of the edges back
                if len(badNodes) > 0:
                    newNode = badNodes.pop()
                    Edge(node_euclidean(start, newNode), start, newNode)
                else:
                    #was unable to find any Node to the goal that was safe :(
                    return None
            else:
                endFound = True
    else:
        end = graphKDTree.getClosestNode(goal_point)

    # The priority of a node will be the length of discovered
    # shortest path from v to the node.
    F = RoadNodeHeap(False)

    # SandF contains the required node information for all nodes in the settled
    # and frontier sets.
    # Keys and RoadNode objects and Values are RoadNodeInfo objects.
    SandF = {}

    # Initialize Settled={}, F={start}, d[start]=0, bckptr[start]=None
    F.add(start, 0.0)
    SandF[start] = RoadNodeInfo(0.0, None)

    # Invariant:
    # (1) For a node s in Settled set S, a shortest v --> s path exists that
    # contains only settled nodes; d[s] is its length (distance) and bk[s] is
    # s's backpointer.
    # (2) For each node f in Frontier set F, a v --> f path exists that contains
    # only settled nodes except for f; d[f] is the length (distance) of the
    # shortest such path and bk[f] is f's backpointer on that path.
    # (3) All edges leaving S go to F.
    while (len(F) != 0):
        # f= node in F with minimum d value. The path to this node is the shortest path.
        f= F.poll()

        # if f is end we have found the route to take.
        if f==end:
            path = []
            while (end != None):
                path.append(end)
                end= SandF[end].bkptr
            # UNCOMMENT TO add path from current_position to start node (closest node)
            # path.append(RoadNode(current_position[0],current_position[1]))
            path.reverse()
            # UNCOMMENT TO add path from end node (node closest to goal) to goal_point
            # path.append(RoadNode(goal_point[0],goal_point[1]))

            # Add the bad edges back to the graph
            while len(badNodes) > 0:
                newNode = badNodes.pop()
                Edge(node_euclidean(start, newNode), start, newNode)
            return path

        fInfo = SandF[f]
        edges = f.edges
        for edge in edges:
            w = edge.getOther(f) #get neighbor for each edge.
            pathLength = fInfo.dist + edge.weight
            if w not in SandF.keys(): # if w is in far off set
                F.add(w, pathLength) # add it to the frontier
                SandF[w] = RoadNodeInfo(pathLength, f)
            elif pathLength < SandF[w].dist: # if w is in F and if newPath<d[w]
                wInfo= SandF[w]              # update priority and info
                wInfo.dist = pathLength
                wInfo.bkptr = f
                F.updatePriority(w, pathLength)

# nodes, edges = get_graph()
# plot_graph(edges)


def unitTestingMap():
    """map used for unit testing"""
    start = RoadNode(0.0, 0.0)
    waypoint1 = RoadNode(-84.0, 0.0)
    waypoint2 = RoadNode(-211.0, -47.0)
    waypoint3 = RoadNode(-211.0, -120.0)
    waypoint3a = RoadNode(-211.0, -127.3)
    waypoint4 = RoadNode(-84.0, -127.3)
    waypoint5 = RoadNode(-81.7, -8.3)
    waypoint6 = RoadNode(34.5, -2.6)
    waypoint7 = RoadNode(-199.7, -254.0)
    waypoint7a = RoadNode(-211.0, -254.0)
    waypoint7b = RoadNode(-199.7, -256.0)
    waypoint8a = RoadNode(-211.0, 0.0)
    waypoint8b = RoadNode(-211.0, -8.1)
    waypoint9 = RoadNode(-93.5, -2.0)
    waypoint10 = RoadNode(-86.0, -119.4)
    waypoint11 = RoadNode(43.9, -127.3)
    waypoint12 = RoadNode(45.9, -9.8)
    waypoint12a = RoadNode(43.9, -9.8)
    waypoint12b = RoadNode(34.5, 0.0)
    waypoint13 = RoadNode(-64.5, 0.0)
    waypoint14 = RoadNode(-84.0, -47.8)
    waypoint15 = RoadNode(-204.2, -47.8)
    waypoint16 = RoadNode(-211.0, -60.0)
    waypoint17 = RoadNode(-84.0, -139.2)
    waypoint18 = RoadNode(-84.0, -256.0)
    waypoint19 = RoadNode(-33.2, -240.4)
    waypoint19a = RoadNode(-33.2, -256.0)
    waypointBLa = RoadNode(40.0, -256.0)
    waypointBLb = RoadNode(43.9, -248.0)

    edge_set = [
    Edge(node_euclidean(start, waypoint13), start, waypoint13),
    Edge(node_euclidean(waypoint13, waypoint1), waypoint13, waypoint1),
    Edge(node_euclidean(waypoint1, waypoint8a), waypoint1, waypoint8a),
    Edge(node_euclidean(waypoint8a, waypoint8b), waypoint8a, waypoint8b),
    Edge(node_euclidean(waypoint8b, waypoint2), waypoint8b, waypoint2),
    Edge(node_euclidean(waypoint2, waypoint16), waypoint2, waypoint16),
    Edge(node_euclidean(waypoint16, waypoint3), waypoint16, waypoint3),
    Edge(node_euclidean(waypoint3, waypoint3a), waypoint3, waypoint3a),
    Edge(node_euclidean(waypoint3a, waypoint7a), waypoint3a, waypoint7a),
    Edge(node_euclidean(waypoint7a, waypoint7b), waypoint7a, waypoint7b),
    Edge(node_euclidean(waypoint7b, waypoint18), waypoint7b, waypoint18),
    Edge(node_euclidean(waypoint18, waypoint19a), waypoint18, waypoint19a),
    Edge(node_euclidean(waypoint19a, waypointBLa), waypoint19a, waypointBLa),
    Edge(node_euclidean(waypointBLa, waypointBLb), waypointBLa, waypointBLb),
    Edge(node_euclidean(waypointBLb, waypoint11), waypointBLb, waypoint11),
    Edge(node_euclidean(waypoint11, waypoint12a), waypoint11, waypoint12a),
    Edge(node_euclidean(waypoint12a, waypoint12b), waypoint12a, waypoint12b),
    Edge(node_euclidean(waypoint12b, start), waypoint12b, start),

    Edge(node_euclidean(waypoint1, waypoint14), waypoint1, waypoint14),
    Edge(node_euclidean(waypoint14, waypoint15), waypoint14, waypoint15),
    Edge(node_euclidean(waypoint15, waypoint2), waypoint15, waypoint2),
    Edge(node_euclidean(waypoint14, waypoint4), waypoint14, waypoint4),
    Edge(node_euclidean(waypoint4, waypoint17), waypoint4, waypoint17),
    Edge(node_euclidean(waypoint17, waypoint18), waypoint17, waypoint18),
    Edge(node_euclidean(waypoint4, waypoint11), waypoint4, waypoint11),
    Edge(node_euclidean(waypoint4, waypoint3a), waypoint4, waypoint3a),

    Edge(node_euclidean(waypoint19a, waypoint19), waypoint19a, waypoint19)
        ]

    node_set =  [waypoint1, waypoint2, waypoint3, waypoint3a, waypoint4, waypoint7a, waypoint7b, waypoint8a, waypoint8b,
                waypoint11, waypoint12a, waypoint12b, waypoint13, waypoint14, waypoint15, waypoint16, waypoint17,
                waypoint18, waypoint19, waypoint19a, waypointBLa, waypointBLb]

    return node_set, edge_set

