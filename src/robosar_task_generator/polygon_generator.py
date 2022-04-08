""" Based on following paper: Free-space Polygon Creation based on Occupancy Grid Maps for Trajectory Optimization Methods"""

import numpy as np
from matplotlib import pyplot as plt

def round_pnt(pnt):
    """
    Rounds given point and returns it as integers
    """
    return np.array([pnt[0],pnt[1]]).astype('int')

def collision_detection(pnts, map, threshold):
    """
    Checks provided list of points (n x 2, int) for collision on map
    Returns collision coord with highest value
    """
    # Check for illegal values
    if(np.any(pnts[:,0] < 0) or np.any(pnts[:,0] > map.shape[0]) or np.any(pnts[:,1] < 0) or np.any(pnts[:,1] > map.shape[1])):
        return True
    else:
        vals = map[pnts[:,0],pnts[:,1]]
        is_collision = np.any(vals > threshold)
        idx = np.argmax(vals)
        pnt = pnts[idx]
        return is_collision, pnt

def plot_points(pnts, color = None):
    # Visualize points
    plt.scatter(pnts[:,0], pnts[:,1], color=color)

def plot_map(map, color = None, threshold=0.5):
    # Visualize map
    xx,yy = np.meshgrid(np.arange(map.shape[0]), np.arange(map.shape[1]))
    mask = map[xx,yy] > threshold
    plt.scatter(xx[mask],yy[mask], color = color)

def plot_polygon(polygon, color = None):
    # Visualize polygon
    temp_pnts = np.vstack((polygon, polygon[0]))
    plt.plot(temp_pnts[:,0], temp_pnts[:,1])

def plot_polygon_list(polygons):
    # Visualize multiple polygons
    for poly in polygons:
        plot_polygon(poly)

def get_line(initial_pnt, angle, max_range):
    """
    Generates a line (n x 2, ordered) from starting point up to max range at 
    specified angle, or until collision occurs
    """
    to_rtn = initial_pnt.reshape(1,2)
    step_size = 0.1
    cur_pnt = initial_pnt.astype('float')
    for i in range(0,int(max_range/step_size)):
        # Compute new point
        cur_pnt[0] = cur_pnt[0] + step_size * np.cos(angle)
        cur_pnt[1] = cur_pnt[1] + step_size * np.sin(angle)
        cur_pnt_int = round_pnt(cur_pnt)
        # Check for redundancy
        if(np.all(cur_pnt_int == to_rtn[-1])):
            continue
        # Append
        to_rtn=np.vstack((to_rtn, cur_pnt_int))            
    return to_rtn

def get_scircle(center, angle, radius):
    """
    Draws semicircle with given radius oriented at angle at center
    center: point, 1 x 2 array int
    angle: radians
    radius: float
    """
    first_time = True
    cur_angle = angle
    resolution = 100
    for i in range(0,resolution):
        cur_angle = angle + (i - resolution/2)/resolution * np.pi
        cur_pnt = np.array([
            center[0] + radius * np.cos(cur_angle),
            center[1] + radius * np.sin(cur_angle)
        ])
        cur_pnt = round_pnt(cur_pnt)
        if(first_time):
            first_time = False
            to_rtn = cur_pnt
        else:
            # Check for redundancy
            if(np.all(cur_pnt == to_rtn[-1])):
                continue
            to_rtn=np.vstack((to_rtn, cur_pnt))
    return to_rtn


def polygon_generator(initial_pnt, initial_angle, map, num_vert, radius, max_range, threshold):
    """
    Generates polygon defining free space inside map starting from provided point
    initial_pnt: (x,y) for initial point; (2,) matrix of ints
    initial_angle: float for initial angle in radians
    map: occupancy grid map; two dimensional
    radius: radius of semi-circle
    max_range: maximum length vertice can be from initial point
    threshold: value above which point on map is considered colliding with semi-circle
    """
    first_time = True
    for vert_idx in range(0, num_vert):
        # Compute angle
        cur_angle = initial_angle + np.pi-2*np.pi*vert_idx/(num_vert + 1)
        # Compute line
        cur_line = get_line(initial_pnt, cur_angle, max_range)
        # Loop over line, then semicircle, checking for collisions
        go_to_next_vert = False
        for line_idx in range(0,cur_line.shape[0]):
            # Compute semicircle
            cur_center = cur_line[line_idx]
            cur_scircle = get_scircle(cur_center, cur_angle, radius)
            # Collision detection
            is_collision, new_pnt = collision_detection(cur_scircle, map, threshold)
            if(is_collision):
                # Collision occurred; append to list then break
                if(first_time):
                    first_time = False
                    poly_list = new_pnt
                else:
                    poly_list = np.vstack((poly_list, new_pnt))
                go_to_next_vert = True
                break
            # If no collision detected, keep going
        # Check if collision occurred; if not, append current point
        if(go_to_next_vert == False):
            if(first_time):
                first_time = False
                poly_list = cur_center
            else:
                poly_list = np.vstack((poly_list, cur_center))
    return poly_list

def multi_polygon_generation(points, initial_angle, map, num_vert, radius, max_range, threshold):
    """
    Takes in multiple points and uses each to generate multiple polygons
    points: (n x 2, int) list of points
    """
    poly_list = []
    for row_idx in range(0,points.shape[0]):
        poly_list.append(polygon_generator(points[row_idx,:], initial_angle, map, num_vert, radius, max_range, threshold))
    return poly_list

if __name__ == "__main__":
    # Generate map; map uses (x,y) not (row,col)
    test_map = np.zeros((300, 400))
    # Boundaries
    test_map[[0,-1],:] = 1
    test_map[:,[0,-1]] = 1
    # Horizontal walls
    test_map[75:125, [100,200,300]] = 1
    test_map[175:, [100,200,300]] = 1
    # Vertical walls
    test_map[[125,175], 75:125] = 1
    test_map[[125,175], 175:225] = 1
    test_map[[125,175], 275:325] = 1
    # Thicken
    test_map = ndimage.binary_dilation(test_map, iterations=4)
    test_map = test_map.astype('int')

    polygen.plot_map(test_map, 'k')
    plt.show()
    
    print("Done")
