import numpy as np
import math
import scipy.stats as st
from nptyping import Array
from pandas._typing import *
from typing import List, Set, Dict, Tuple, Optional


class SampleRectangleError(Exception):
    """Not entirely sure why I made this exception tbh
  """

    def __init__(self, message, *args):
        super().__init__(message)
        self.args = [a for a in args]


class Rectangle:
    """A class representing a rectangle that is parallel to the x and y axes.
  """

    # def __init__(self, pt1: Tuple[int, int], pt2: Tuple[int, int]) -> None:
    def __init__(self, pt1, pt2):
        self._x1: float = min(pt1[0], pt2[0])
        self._x2: float = max(pt1[0], pt2[0])
        self._y1: float = min(pt1[1], pt2[1])
        self._y2: float = max(pt1[1], pt2[1])
        self._w: float = self._x2 - self._x1
        self._h: float = self._y2 - self._y1
        self._area: float = self._w * self._h

    @property
    def x1(self):
        """Left corner x coordinate"""
        return self._x1

    @property
    def x2(self):
        """Right corner x coordinate"""
        return self._x2

    @property
    def y1(self):
        """Bottom corner y coordinate"""
        return self._y1

    @property
    def y2(self):
        """Top corner y coordinate"""
        return self._y2

    @property
    def w(self):
        """Rectangel width"""
        return self._w

    @property
    def h(self):
        """Rectangle height"""
        return self._h

    @property
    def area(self):
        """Rectangle Area"""
        return self._area


def rand_rect(rect: Rectangle, num_points: int) -> Tuple[Array[float], Array[float]]:
    """Generates [num_points] number of pseudo-randomly sampled points inside [rect]"""
    px = np.random.rand(num_points)
    py = np.random.rand(num_points)
    rx = np.array([rect.x2 - rect.x1])
    ry = np.array([rect.y2 - rect.y1])
    px *= rx
    py *= ry
    return px + rect.x1, py + rect.y1


def is_in_rect(rect: Rectangle, x: Array[float], y: Array[float]) -> Array[bool]:
    """Determines whether points specified by x and y coordinate vectors are in [rect]"""
    a = x - rect.x1
    b = y - rect.y1
    xt = ((a >= 0) & (a <= (rect.x2 - rect.x1)))
    yt = ((b >= 0) & (b <= (rect.y2 - rect.y1)))
    return np.logical_and(xt, yt)


def not_in_rect(rect: Rectangle, x: Array[float], y: Array[float]) -> Array[bool]:
    """Determines whether points specified by x and y coordinate vectors are not in [rect]"""
    a = x - rect.x1
    b = y - rect.y1
    xt = ((a < 0) | (a > (rect.x2 - rect.x1)))
    yt = ((b < 0) | (b > (rect.y2 - rect.y1)))
    return np.logical_or(xt, yt)


def not_in_obs(obstacles: List[Rectangle], x: Array[float], y: Array[float]) -> Array[bool]:
    """Determines whether points specified by x and y coordinate vectors do not collide with the obstacles."""
    result = np.full((x.size), True)
    for o in obstacles:
        result &= not_in_rect(o, x, y)
    # print(result)
    return result


def intersect_area(rect1: Rectangle, rect2: Rectangle) -> float:
    """Area of intersection between [rect1] and [rect2]."""
    w = max((min(rect1.x2, rect2.x2) - max(rect1.x1, rect2.x1)), 0)
    h = max((min(rect1.y2, rect2.y2) - max(rect1.y1, rect2.y1)), 0)
    return w * h


def sample_points_not_in_obs(window: Rectangle, obstacles: List[Rectangle], num_points: int) -> Tuple[
    Array[float], Array[float]]:
    """ Pseudo-randomly generates [num_points] number of points inside [window] that do not collide with obstacles."""
    total_intersect = 0
    for o in obstacles:
        total_intersect += intersect_area(o, window)
    p = 1 - total_intersect / window.area  # TODO: What if the obstacles intersect?
    if p <= 0:
        raise SampleRectangleError("Invalid Sample Space - no free sampling space", window, obstacles, num_points)

    samples = math.ceil(1 / p * num_points * 5)
    ptx, pty = rand_rect(window, samples)
    is_valid = not_in_obs(obstacles, ptx, pty)
    num_valid = np.sum(is_valid.astype(int))
    if num_valid >= num_points:
        ptx, pty = ptx[is_valid], pty[is_valid]
        # print("No additional point generation needed")
    else:
        ptx, pty = ptx[is_valid], pty[is_valid]
        while num_valid < num_points:
            ix, iy = rand_rect(window, math.ceil((num_points - num_valid) * 1 / p * 5))
            is_valid = not_in_obs(obstacles, ix, iy)
            num_valid += np.sum(is_valid.astype(int))
            ptx = np.concatenate((ptx, ix[is_valid]), axis=None)
            pty = np.concatenate((pty, iy[is_valid]), axis=None)

    return ptx[:num_points], pty[:num_points]


########################################################################################################################

def rect_to_array(rectangles: List[Rectangle]) -> Array[float, ..., 4]:
    """ Converts Rectangle objects to the array representation of obstacles: [x, y, width, height]. """
    rect_array = np.zeros((len(rectangles), 4))
    for i, rect in enumerate(rectangles):
        rect_array[i][0], rect_array[i][1], rect_array[i][2], rect_array[i][3] = (
        rect.x1 + rect.w / 2.0, rect.y1 + rect.h / 2.0, rect.w, rect.h)
    return rect_array


def sample_rect_array(rect: Array[float, 4], num_points: int) -> Tuple[Array[float], Array[float]]:
    """Generates [num_points] number of pseudo-randomly sampled points inside [rect]"""
    px = np.random.rand(num_points)
    py = np.random.rand(num_points)
    px *= rect[2]
    py *= rect[3]
    return px + rect[0] - rect[2] / 2.0, py + rect[1] - rect[3] / 2.0

def not_in_obs_array(obstacles, x, y, checkForAnd = False):
    """Determines whether points specified by x and y coordinate vectors do not collide with the obstacles."""
    # [center_x, center_y, width, height]
    # get x coordinates of obstacle centers
    num_obstacles = len(obstacles)
    obstacles_x = obstacles[:, 0]
    obstacles_x = obstacles_x.reshape((1, num_obstacles))    # reshape into 2D array
    obstacles_x = obstacles_x.repeat(len(x), axis=0)   # Add obstacle x vector for each point in x
    diff_x = obstacles_x - x.reshape((len(x), 1))  # Subtract point x coordinates
    diff_x = np.absolute(diff_x)
    max_diff_x = np.absolute(obstacles[:, 2] / 2.0)
    valid_x = np.greater(diff_x, max_diff_x)

    # get x coordinates of obstacle centers
    obstacles_y = obstacles[:, 1]
    obstacles_y = obstacles_y.reshape((1, num_obstacles))  # reshape into 2D array
    obstacles_y = obstacles_y.repeat(len(y), axis=0)  # Add obstacle x vector for each point in x
    diff_y = obstacles_y - y.reshape((len(y), 1))  # Subtract point x coordinates
    diff_y = np.absolute(diff_y)
    max_diff_y = np.absolute(obstacles[:, 3] / 2.0)
    valid_y = np.greater(diff_y, max_diff_y)


    valid_x_y = np.logical_or(valid_x, valid_y)
    valid_x_y = np.prod(valid_x_y, 1, dtype=bool)

      # Combine result from each obstacles into a vector
    return valid_x_y

def in_circle_array(circles, x, y):
    if len(circles.shape) == 1:
        circles = np.expand_dims(circles, 0)

    num_points = len(x)
    # get x coordinates of obstacle centers
    circles_array = circles[:, 0:2].T
    circles_array = np.expand_dims(circles_array, -1).repeat(num_points, -1)

    points_array = np.stack([x, y]).reshape(2, 1, -1)
    point_offset_from_center = circles_array - points_array
    distance_from_center = point_offset_from_center[0] ** 2. + point_offset_from_center[1] ** 2.

    # Collect circle radii
    circle_radii = circles[:, 2]
    within_radius = distance_from_center < (circle_radii ** 2.).reshape(-1, 1)
    return np.sum(within_radius, 0) > 0

def not_in_obs_matrix(obstacles: Array[float,...,4], spline_matrix: Array[float]) -> Array[bool]:
    # spline_matrix is a 3D array of shape (number_of_splines, 2, number_of_spline_points)

    obstacles_x = obstacles[:, 0]   # One Dimensional
    obstacles_y = obstacles[:, 1]
    obstacle_array = np.stack(obstacles_x, obstacles_y)
    spline_offset_from_center = np.absolute(spline_matrix - obstacle_array)  # Distance from obstacle center

    max_offset = np.stack(obstacles[:, 2] / 2.0, obstacles[:, 3])
    valid_offset_from_center = np.greater(spline_offset_from_center, max_offset)
    valid_splines = np.logical_or(valid_offset_from_center[:,0,:], valid_offset_from_center[:,1,:])
    valid_splines = np.prod(valid_splines, 1, dtype=bool)

    return valid_splines

def intersect_with_window_area(rectangles: Array[float,...,4], window: Array[float,4]) -> float:
    """Area of intersection between [rectangles]."""
    # TODO: Not trivial at all, look into intersection trees
    rectangles_x1 = rectangles[:, 0] - rectangles[:, 2] / 2.0
    rectangles_x2 = rectangles[:, 0] + rectangles[:, 2] / 2.0
    rectangles_y1 = rectangles[:, 1] - rectangles[:, 3] / 2.0
    rectangles_y2 = rectangles[:, 1] + rectangles[:, 3] / 2.0
    window_x1 = np.full(rectangles_x1.size, window[0] - window[2] / 2.0)
    window_x2 = np.full(rectangles_x1.size, window[0] + window[2] / 2.0)
    window_y1 = np.full(rectangles_y1.size, window[1] - window[3] / 2.0)
    window_y2 = np.full(rectangles_y1.size, window[1] + window[3] / 2.0)

    w = np.maximum((np.minimum(window_x2, rectangles_x2) - np.maximum(window_x1, rectangles_x1)), np.zeros(window_x2.size))
    h = np.maximum((np.minimum(window_y2, rectangles_y2) - np.maximum(window_y1, rectangles_y1)), np.zeros(window_y2.size))
    return np.dot(w,h)

def total_rectangle_area(rectangles: Array[float,...,4]) -> float:
    return np.dot(rectangles[:, 2], rectangles[:, 3])    # TODO: This is naive

def sample_points_not_in_obs_array(window: Array[float,4], obstacles: Array[float,...,4], num_points: int) -> Tuple[
    Array[float], Array[float]]:
    """ Pseudo-randomly generates [num_points] number of points inside [window] that do not collide with obstacles."""
    total_area = intersect_with_window_area(obstacles, window)
    p = 1 - total_area / (window[2] * window[3])  # TODO: What if the obstacles intersect?
    if p <= 0:
        raise ValueError("Invalid Sample Space - no free sampling space", window, obstacles, num_points)

    samples = math.ceil(1 / p * num_points * 3)
    ptx, pty = sample_rect_array(window, samples)
    is_valid = not_in_obs_array(obstacles, ptx, pty)
    num_valid = np.sum(is_valid.astype(int))
    if num_valid >= num_points:
        ptx, pty = ptx[is_valid], pty[is_valid]
        # print("No additional point generation needed")
    else:
        ptx, pty = ptx[is_valid], pty[is_valid]
        while num_valid < num_points:
            ix, iy = sample_rect_array(window, math.ceil((num_points - num_valid) * 1 / p * 5))
            is_valid = not_in_obs_array(obstacles, ix, iy)
            num_valid += np.sum(is_valid.astype(int))
            ptx = np.concatenate((ptx, ix[is_valid]), axis=None)
            pty = np.concatenate((pty, iy[is_valid]), axis=None)

    return ptx[:num_points], pty[:num_points]


def sample_free_points(window, obstacles, num_points, confidence=0.99):
    """ Samples with [confidence] probability [num_points] number of points inside [window] that do not collide with obstacles. """
    obstacle_area = intersect_with_window_area(obstacles, window)
    p = 1.0 - obstacle_area / (window[2] * window[3])    # TODO: What if the obstacles intersect?
    z_score = st.norm.ppf(confidence)
    roots = np.roots(
        [p**2.0, z_score**2.0 * p**2.0 - 2.0*num_points*p - z_score**2.0*p, num_points**2])
    real_valued = roots.real[abs(roots.imag) < 1e-5]
    n = int(real_valued[0] + 0.5)
    ptx, pty = sample_rect_array(window, n)
    is_valid = not_in_obs_array(obstacles, ptx, pty)
    num_valid = np.sum(is_valid.astype(int))
    ptx, pty = ptx[is_valid], pty[is_valid]
    if num_valid >= num_points:
        return ptx[:num_points], pty[:num_points]
    return ptx, pty

# circles = np.asarray([[5, 5, 3], [-7, 3, 3], [-8, 0, 1]])
# points = np.asarray([[5, 5], [7, 7], [10, 10], [-8, 0.5]])
# x = points[:, 0]
# y = points[:, 1]
# print(in_circle_array(circles, x, y))