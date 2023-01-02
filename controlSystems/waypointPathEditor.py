#!/usr/bin/env python
import MainConfigClass

def buildOptimalPath(pathList, speeds):
    """build the linked list object, doubly-linked non-circular"""

    head = None
    linkedListHead = waypoint(head, pathList[0], None, speeds[0])
    head = linkedListHead
    newWayPoint = None

    count = 1
    for point in pathList[1:]:
        newWayPoint = waypoint(head, point, None, speeds[count])
        head.tail = newWayPoint
        head = newWayPoint
        count += 1
    linkedListTail = newWayPoint

    return linkedListHead, linkedListTail

def buildPath(pathList):
    """build the linked list object, doubly-linked non-circular"""

    head = None
    linkedListHead = waypoint(head, pathList[0], None)
    head = linkedListHead
    newWayPoint = None

    for point in pathList[1:]:
        newWayPoint = waypoint(head, point, None)
        head.tail = newWayPoint
        head = newWayPoint

    linkedListTail = newWayPoint

    return linkedListHead, linkedListTail

def suturePath(head, tail): #change name?
    """links two parts of a linked list together"""
    head.tail = tail
    tail.head = head

def implantPath(implantHead, implantTail, startTail, endHead):
    """implants a path into the map and links it to a certain waypoint, oldTail is safeEnd in hybridSplineRouter"""
    suturePath(startTail, implantHead)
    suturePath(implantTail, endHead)

def printPath(path):
    """print out the path information, only for non-circular linked lists"""
    while path.tail is not None:
        point = path.point
        print('x:', point.x, 'y:', point.y, 'implanted:', path.implanted)
        path = path.tail

class waypoint(object):
    """doubly linked list object for nodes"""

    def __init__(self, head, point, tail, velocity = MainConfigClass.IGNORE_VELOCITY, implanted=False):
        self.head = head
        self.point = point
        self.velocity = velocity
        self.tail = tail
        self.implanted = implanted #if false, send a special message .000111000111000, otherwise send point.velocity in the [x, y, v]
