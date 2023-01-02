import numpy as np
import time

def apply_bezier_to_path(path_points, step):
    start_time = time.time()
    bezier_points = []  # points in the bezier curve
    bezier_edges = np.flip(path_points, -1).T  # points in pathPoints to connect in forming the bezier curve
    for k in range(len(bezier_edges) - 3):
        if k % 3 == 0:
            add_curve(bezier_points, bezier_edges[k][0], bezier_edges[k][1], bezier_edges[k + 1][0], bezier_edges[k +
            1][1],
                      bezier_edges[k + 2][0], bezier_edges[k + 2][1], bezier_edges[k + 3][0], bezier_edges[k + 3][1], step,
                      'bezier')
    bezier_time = time.time() - start_time
    print("Bezier smoothing time: ", bezier_time)
    return bezier_points

def add_curve(points, x0, y0, x1, y1, x2, y2, x3, y3, step, curve_type ):
    t = 0
    while (t < 1):
        x_matrix = generate_curve_coefs(x0, x1, x2, x3, curve_type)
        #print x_matrix
        y_matrix = generate_curve_coefs(y0, y1, y2, y3, curve_type)
        #print y_matrix
        x = x_matrix[0][0] * (t ** 3) + x_matrix[0][1] * (t ** 2) + x_matrix[0][2] * t + x_matrix[0][3]
        y = y_matrix[0][0] * (t ** 3) + y_matrix[0][1] * (t ** 2) + y_matrix[0][2] * t + y_matrix[0][3]
        add_point(points, x, y)
        #points.append([x, y, 0, 1])
        t += step

def add_point( matrix, x, y, z=0 ):
    matrix.append( [x, y, z, 1] )

def make_bezier():
    t = new_matrix()
    t[0][0] = -1
    t[0][1] = 3
    t[0][2] = -3
    t[0][3] = 1
    t[1][0] = 3
    t[1][1] = -6
    t[1][2] = 3
    t[2][0] = -3
    t[2][1] = 3
    t[3][0] = 1
    return t

def generate_curve_coefs( p1, p2, p3, p4, t ):
    #m2 = [[p1],[p2],[p3],[p4]]
    m2 = new_matrix(4,1)
    #print m2
    if (t == 'hermite'):
        m1 = make_hermite()
    elif (t == 'bezier'):
        m1 = make_bezier()
    m2[0][0] = p1
    m2[0][1] = p2
    m2[0][2] = p3
    m2[0][3] = p4
    matrix_mult(m1, m2)
    #print m2
    return m2

def matrix_mult( m1, m2 ):

    point = 0
    for row in m2:
        #get a copy of the next point
        tmp = row[:]

        for r in range(4):
            m2[point][r] = (m1[0][r] * tmp[0] +
                            m1[1][r] * tmp[1] +
                            m1[2][r] * tmp[2] +
                            m1[3][r] * tmp[3])
        point+= 1


def new_matrix(rows = 4, cols = 4):
    m = []
    for c in range( cols ):
        m.append( [] )
        for r in range( rows ):
            m[c].append( 0 )
    return m