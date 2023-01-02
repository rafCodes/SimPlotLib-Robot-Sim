import numpy as np
import math

class TrackNode:
    """
    Class representing a track feature as a Node of a 3D coordinate
    """
    def __init__(self, x, y, z):
        self.edgesOut = []  # Contains edges from this node
        self.edgesIn = []     # Contains edges to this node
        self.neighborsFrom = []   # Contains nodes from which you can get here
        self.neighborsTo = [] # Contains nodes to which you can go from here
        self.x = x
        self.y = y
        self.z = z

    def addEdgeIn(self, edge):
        """ Add an edge to this Node """
        assert edge.nodeTo == self
        self.edgesIn.append(edge)
        if edge.nodeFrom not in self.neighborsFrom:
            self.neighborsFrom.append(edge.nodeFrom)

    def addEdgeOut(self, edge):
        """ Add an edge to this Node """
        assert edge.nodeFrom == self
        self.edgesOut.append(edge)
        if edge.nodeTo not in self.neighborsTo:
            self.neighborsTo.append(edge.nodeTo)

    # def findEdgesTo(self, other_node):
    #     """ Returns a list of edges connecting this node and [other_node] """
    #     return filter(lambda n: other_node in n.nodes, self.edges)

    # def __str__(self):
    #     return str((self.x, self.y, self.z))


class Edge:
    """
    Class representing a directed weighted edge
    """

    def __init__(self, weight, nodeFrom, nodeTo):
        assert isinstance(nodeFrom, TrackNode) and isinstance(nodeTo, TrackNode)
        assert type(weight) == float or type(weight) == int
        assert nodeFrom != nodeTo
        self.weight = weight
        self.nodeFrom = nodeFrom
        self.nodeTo = nodeTo
        nodeFrom.addEdgeOut(self)
        nodeTo.addEdgeIn(self)

    # def getOther(self,node):
    #     """
    #     Return the other node of the edge.
    #
    #     Parameter node: Node where this edge 'begins'
    #     Precondition: This edge is connected to this Node.
    #     """
    #     assert node in self.nodes
    #     if node==self.nodes[0]:
    #         return self.nodes[1]
    #     return self.nodes[0]
