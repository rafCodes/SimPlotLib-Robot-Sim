import numpy as np
import math
import scipy.optimize as opt

from optimization.TrackNodeHeap import *
from optimization.TrackNodeKDTree import *
from optimization.TrackGraph import *

############################### Main Function #################################
def optimumDrive(graphKDTree, start, finish, mass, gravity, dragC, rollingC, timeLimit, timeLimitWeight):
    """
    Main function to be called in real-time
    """
    path, en = optimumPath(graphKDTree, start, finish)
    n = len(path)
    X = np.zeros(n); Y = np.zeros(n); Z = np.zeros(n)
    for i in range(n):
        node = path[i]
        X[i] = node.x
        Y[i] = node.y
        Z[i] = node.z
    speeds = getBestProfile(X, Y, Z, mass, gravity, dragC, rollingC, timeLimit, timeLimitWeight)
    return path, speeds

############################### Graph Creation #################################
def createTrackGraphAndTree(inner, outer, clockwise, mass, gravity, dragC, rollingC, avgV):
    """
    Initializer code to create TrackGraph and KDTree from inner and outer data of
    track.
    """
    graph = createGraph(interpolate(inner, outer), clockwise, mass, gravity, dragC, rollingC, avgV)
    kdtree = TrackNodeKDTree(graph)
    return graph, kdtree

def interpolate(innerData, outerData):
    """
    Returns a list of arrays of row vectors in the form of [Inner Track,
    First Quarter, Middle Track, Second Quarter, Outer Track]

    Parameter innerData: The datapoints for the inner border of the track.
    Precondition: innerData is a list of row vectors with same length as outerData.

    Parameter outerData: The datapoints for the outer border of the track.
    Precondition: outerData is a list of row vectors  with same length as innerData.
    """
    assert len(innerData) == len(outerData)
    assert len(innerData) > 0
    assert len(innerData[0]) == len(outerData[0])
    IN = np.array(innerData)
    OUT = np.array(outerData)
    MID = (IN + OUT)/2
    FQ = (IN + MID)/2
    SQ = (MID + OUT)/2
    return [IN, FQ, MID, SQ, OUT]

def createGraph(arrayList, clockwise, mass, gravity, dragC, rollingC, avgV):
    """
    Returns a list of Track Nodes after creating the relevant edges from the
    array list.

    Parameter arrayList: The datapoints from which to create the graphs
    Precondition: arrayList is a list of matrices with row vectors where the ith
    row vector of a matrix can have edges to the (i+1)th row vectors or (i-1)th
    row vectors of all the matrices if clockwise is true or false respectively.

    Optional Parameter clockwise: Traversal direction is clockwise or not.
    Precondition: clockwise is a bool.
    """
    assert len(arrayList) > 0 and type(clockwise)==bool
    numArrs = len(arrayList)
    arrLen = len(arrayList[0])
    graph = [] # node corresponding arrayList[h][k] = graph[k*numArrs+h]
    for i in range(arrLen):
        for points in arrayList:
            node = TrackNode(points[i][0], points[i][1], points[i][2])
            graph.append(node)
            if i != 0:
                for index in range(numArrs):
                    fromNode = graph[(i-1)*numArrs+index]
                    weight = energy(fromNode, node, avgV, mass, gravity, dragC, rollingC)
                    Edge(weight, fromNode, node)
    for points in arrayList:
        for index in range(numArrs):
            fromNode = graph[(arrLen-1)*numArrs+index]
            toNode = graph[index]
            weight = energy(fromNode, toNode, avgV, mass, gravity, dragC, rollingC)
            Edge(weight, fromNode, toNode)
    return graph

############################### Energy functions ###############################

def energy(a, b, v, m, g, CoeffAR, CoeffRR):
    """
    Returns the energy required for a vehicle to go from TrackNode a to b at a
    constant speed.
    """
    dist = math.sqrt((b.x - a.x)**2+(b.y - a.y)**2+(b.z - a.z)**2)
    va = math.asin((b.z - a.z)/dist)
    force = CoeffAR*v**2 + CoeffRR*m*g*math.cos(va) + m*g*math.sin(va) # + CoeffCR*sa
    energy = dist*force
    if energy < 0:
        energy = 0
    return energy

def energyArrayWise(SP, D, sinVA, m, g, ca, cr):
    """
    Returns the cost of a given speed profile.
    """
    V = np.zeros(len(SP)+2)
    V[1:-1] = SP
    # V = SP
    avgV = (V[1:]+V[:-1])/2
    F = m*(V[1:]**2-V[:-1]**2)/(2*D) + m*g*sinVA + ca*avgV**2 + cr*m*g*np.cos(np.arcsin(sinVA))
    F = np.where(F>0, F, 0)
    return np.dot(F, D)

def energyArrayWiseSanityTest():
    #TB Added
    X = np.array([0, 1, 0]); Y = np.array([0, 0, 0]); Z=np.array([0, 0, 0]); SP=np.array([1])
    assert energy(X, Y, Z, SP, 1, 10, 0, 0)==0.5, "Expected: 0.5 but was "+str(energy(X, Y, Z, SP, 1, 10, 0, 0))
    X = np.array([0, 1, 2]); Y = np.array([0, 0, 0]); Z=np.array([0, 0, 0]); SP=np.array([1])
    assert energy(X, Y, Z, SP, 1, 10, 0, 0)==0.5, "Expected: 0.5 but was "+str(energy(X, Y, Z, SP, 1, 10, 0, 0))
    X = np.array([0, 1, 2]); Y = np.array([0, 0, 0]); Z=np.array([0, 2, 0]); SP=np.array([1])
    assert energy(X, Y, Z, SP, 1, 10, 0, 0)==20.5, "Expected: 20.5 but was "+str(energy(X, Y, Z, SP, 1, 10, 0, 0))

##################### Approach 1: Djikstra's then Minimize ######################

def getBestProfile(X, Y, Z, m, g, ca, cr, maxTime, k):
    """
    Returns the cost minimizing speed profile for a given path.
    """
    def cost(sp):
        dX = X[1:]-X[:-1]
        dY = Y[1:]-Y[:-1]
        dZ = Z[1:]-Z[:-1]
        D = np.sqrt(dX**2+dY**2+dZ**2)
        sinVA = dZ/D
        return energyArrayWise(sp, D, sinVA, m, g, ca, cr) + k*(np.sum(D[1:]/sp)-maxTime)**2
    return opt.minimize(cost, x0=np.array([5]*(len(X)-2))).x

def optimumPath(graphKDTree, current_position, goal_point):
    """
    Returns the cheapest path following track graph nodes from start to end.
    """
    # IF you want to find nodes closest to start and end points
    start = graphKDTree.getClosestNode(current_position)
    end = graphKDTree.getClosestNode(goal_point)

    # The priority of a node will be the length of discovered
    # shortest path from v to the node.
    F= TrackNodeHeap(False);

    # SandF contains the required node information for all nodes in the settled
    # and frontier sets.
    # Keys and TrackNode objects and Values are TrackNodeInfo objects.
    SandF = {}

    # Initialize Settled={}, F={start}, d[start]=0, bckptr[start]=None
    F.add(start, 0.0);
    SandF[start]= TrackNodeInfo(0.0, None);

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
            path = []; energy = SandF[end].dist
            while (end != None):
                path.append(end)
                end= SandF[end].bkptr
            # UNCOMMENT TO add path from current_position to start node (closest node)
            # path.append(TrackNode(current_position[0],current_position[1]))
            path.reverse()
            # UNCOMMENT TO add path from end node (node closest to goal) to goal_point
            # path.append(TrackNode(goal_point[0],goal_point[1]))
            return path, energy
        fInfo= SandF[f]
        edges= f.edgesOut
        for edge in edges:
            w= edge.nodeTo #get neighbor for each edge.
            pathLength= fInfo.dist + edge.weight
            if w not in SandF.keys(): # if w is in far off set
                F.add(w, pathLength) # add it to the frontier
                SandF[w]= TrackNodeInfo(pathLength, f)
            elif pathLength < SandF[w].dist: # if w is in F and if newPath<d[w]
                wInfo= SandF[w]              # update priority and info
                wInfo.dist= pathLength
                wInfo.bkptr= f
                F.updatePriority(w, pathLength)

################### Approach 2: Recursion with Memoization #####################
def costPointWise(a, b, v, k=5):
    """
    Returns the cost vehicle incurs in going from TrackNode a to b at v speed.
    """
    dist = math.sqrt((b.x - a.x)**2+(b.y - a.y)**2+(b.z - a.z)**2)
    return energy(a, b, v) + k*(dist/v-dist/5)**2

def optimize(start, end, velocities, visited, mem):
    if start==end:
        return [], [], 0
    if start in mem:
        return mem[start]
    visited.add(start)
    res = []
    spres = []
    ener = float('inf')
    for e in start.edgesOut:
        for v in velocities:
            n = e.nodeTo
            if n not in visited:
                subres, sp, subener = optimize(n, end, velocities, visited, mem)
                subener += costPointWise(start, n, v)
                if subener<ener:
                    res = [start] + subres
                    spres = [v] + sp
                    ener = subener
    visited.remove(start)
    mem[start] = (res, spres, ener)
    return res, spres, ener


####################### Node Info Class for Djikstra's #########################

class TrackNodeInfo():
    """
    Class to store information regarding the shortest path of various TrackNodes.
    """
    # _dist= shortest known distance from the start node to this one
    # _bckptr= backpointer on path (with shortest known distance)
    #           from start node to this one

    @property
    def dist(self):
        return self._dist

    @property
    def bckptr(self):
        return self._bckptr

    @bckptr.setter
    def bckptr(self,bp):
        assert isinstance(bp,TrackNode)
        self._bckptr = bp

    @dist.setter
    def dist(self,d):
        assert type(d)==float
        self._dist = d

    def __init__(self, dist, bp):
        """
        Initializes an TrackNodeInfo with distance from the start node and a
        backpointer.

        Parameter dist: shortest known distance from the start node to this one
        Precondition: dist is a float

        Parameter bp: backpointer on path (with shortest known distance)
                        from start node to this one
        Precondition: bp is a TrackNode
        """
        self.dist= dist
        self.bkptr= bp

    def __str__(self):
        """
        Overrides python function str(TrackNodeInfo)
        """
        return "distance" + str(self._dist) + ", backpointer " + str(self._bkptr)
