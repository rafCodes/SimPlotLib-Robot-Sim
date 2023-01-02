import math
import random
import time
import numpy as np
import pandas as pd

import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from matplotlib.widgets import Button

from optimizer import createTrackGraphAndTree, optimumDrive

mass = 96
gravity = 9.8
dragC = 0.01
rollingC = 0.01
timeLimit = 10
timeLimitWeight = 10
avgV = 5
clockwise = True

# x=trackData(:,1).* 111000;
# y=trackData(:,2).* cosd(trackData(:,1))*111321;
# z=(trackData(:,3).^2+trackData(:,4).^2+trackData(:,5).^2).^(1/2);

#way to make data
def cosd(angle):
    """
    Returns the cosine of an angle in degrees.

    Parameter angle: The cosine of which to find.
    Precondition: angle is a number.
    """
    assert type(angle)==int or type(angle)==float
    return math.cos(math.radians(angle))


def sind(angle):
    """
    Returns the sine of an angle in degrees.

    Parameter angle: The sine of which to find.
    Precondition: angle is a number.
    """
    assert type(angle)==int or type(angle)==float
    return math.sin(math.radians(angle))

def makeData():
    """makes data for the optimizer to use as a inner, outer plot"""
    inside = [];
    out = [];
    for i in range(0, 361, 10):
        inside.append([cosd(i), sind(i), sind(i)])
    for i in range(0, 361, 10):
        out.append([5 * cosd(i), 5 * sind(i), cosd(i)])
    inside.reverse();
    out.reverse()
    return inside, out

inside, out = makeData()

class PlotGraph:

    def __init__(self, graph, tree):
        """
        """
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        x=[node.x for node in graph]
        y=[node.y for node in graph]
        z=[node.z for node in graph]
        ax.plot_wireframe(np.array([x]), np.array([y]), np.array([z]), zorder=1, color='gray')
        ax.set_xlim(min(x)-0.1,max(x)+0.1)
        ax.set_ylim(min(y)-0.1,max(y)+0.1)
        ax.set_zlim(min(z)-0.1,max(z)+0.1)
        ax.set_xlabel('x - axis')
        ax.set_ylabel('y - axis')
        ax.set_zlabel('z - axis')
        ax.view_init(60, 35)
        plt.title('Track')
        self.kdTree = tree
        self.firstPoint = None
        self.ax = ax
        self.fig = fig
        cid = fig.canvas.mpl_connect('button_press_event', self.onclick)
        plt.show()


    def onclick(self,event):
        b = self.ax.button_pressed
        self.ax.button_pressed = -1
        s = self.ax.format_coord(event.xdata,event.ydata)
        self.ax.button_pressed = b
        out = ""
        for i in range(s.find('x')+2,s.find('y')-2):
            out = out+s[i]
        if ord(out[0])==8722 or out[0] in ["-", '\U00002013']:
            xdata = -float(out[1:])
        else:
            xdata = float(out)
        out = ""
        for i in range(s.find('y')+2,s.find('z')-2):
            out = out+s[i]
        if ord(out[0])==8722 or out[0] in ["-", '\U00002013']:
            ydata = -float(out[1:])
        else:
            ydata = float(out)
        print(xdata,ydata)
        if self.firstPoint is None:
            self.firstPoint = (xdata, ydata)
            print("Awaiting second point...")
        else:
            path, speed = optimumDrive(self.kdTree, self.firstPoint, (xdata, ydata), mass, gravity, dragC, rollingC, timeLimit, timeLimitWeight)
            plt.ion()
            X = np.array([a.x for a in path]);
            Y = np.array([a.y for a in path]);
            Z = np.array([a.z for a in path])
            self.ax.plot(X, Y, Z, linewidth=2, zorder=2, c="b")
            self.firstPoint = None
            pt, = self.ax.plot([X[0]], [Y[0]], [Z[0]], marker='o', c='orange')
            self.go(X, Y, Z, speed)
            print("Energy optimum path at average speed displayed.")

    def go(self, X, Y, Z, sp):
        fig, axs = plt.subplots(2)
        dist = []
        height = []
        speed = []
        s = math.sqrt(X[0]**2+Y[0]**2)
        for i in range(1, len(X)):
            dist.append(s)
            height.append(Z[i-1])
            s+=math.sqrt((X[i-1]-X[i])**2+(Y[i-1]-Y[i])**2)
        dist.append(s)
        height.append(Z[i-1])
        speed = [0] + list(sp) + [0]
        assert len(dist)==len(speed)
        axs[0].plot(dist, height)
        axs[1].plot(dist, speed)
        plt.show()

graph, tree = createTrackGraphAndTree(inside, out, clockwise, mass, gravity, dragC, rollingC, avgV)
PlotGraph(graph, tree)

#print(energy(TrackNode(0, 0 , 0), TrackNode(1, 1, 1), 1))
