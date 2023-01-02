import numpy as np

CIRCLE = "circle"
RECTANGLE = "rectangle"

class rrtConfig(object):

    def __init__(self):
        self.NUMNODES = 5000    #  Number of nodes rrt will create before terminating
        self.TIMES_TO_RUN = 1
        self.INITIAL_POINT = np.array([0, 0])
        self.SEARCH_WINDOW = np.array([0, 17.5, 40, 45])
        self.GOAL_REGION = np.array([-10.0, 30.0, 3.0])
        self.MAX_STEER = 30.0 / 180.0 * np.pi
        self.COLLISION_SAMPLE_RATE = 40 # Done before smoothing, so after smoothing the path could violate collision
        self.STEP_DELTA = 2.0   # RRT step granularity. The lower the more nodes are needed to explore space (but denser graph)
        self.GOAL_SHAPE = CIRCLE
        self.NUM_CIRCLE_CHECK_POINTS = 10 # Controls how many straight line paths are checked for circular goals
        self.BEZIER_STEP = 0.2  # Smoothing step