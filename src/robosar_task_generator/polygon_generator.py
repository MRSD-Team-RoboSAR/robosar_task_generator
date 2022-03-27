""" Based on following paper: Free-space Polygon Creation based on Occupancy Grid Maps for Trajectory Optimization Methods"""

import numpy as np
from matplotlib import pyplot as plt

def round_pnt(pnt):
    """
    Rounds given point and returns it as integers
    """
    return np.array([pnt[0],pnt[1]]).astype('int')

def get_line(initial_pnt, angle, map, max_range, threshold):
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
        # Check map; break if collision
        cur_val = map[cur_pnt_int[0], cur_pnt_int[1]]
        if(cur_val > threshold):
            break
            
    plt.scatter(to_rtn[:,0], to_rtn[:,1])
    plt.show()


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
    for vert_idx in range(0, num_vert):
        # Compute angle
        cur_angle = initial_angle + np.pi-2*np.pi*vert_idx/(num_vert + 1)
        # Compute line
        cur_line = np.zeros((5,2)).astype('int')
        # Loop over line, then semicircle, checking for collisions
        go_to_next_vert = False
        for line_idx in range(0,cur_line.shape[0]):
            # Compute semicircle
            center_x, center_y = cur_line[line_idx]
            cur_scircle = np.zeros((5,2)).astype('int')
            for scircle_idx in range(0,cur_scircle.shape[0]):
                # Check for collision
                x_coord, y_coord = cur_scircle[scircle_idx]
                cur_val = map[x_coord, y_coord]
                if(cur_val > threshold):
                    # Collision; append and go to next vert
                    print("Collision")
                    go_to_next_vert = True
                    break
            if(go_to_next_vert):
                # Collision, already accounted for; go to next vert
                break
            else:
                # No collision occurred; append and go to next vert
                print("No collision")
    print("Done")

if __name__ == "__main__":
    # # Generate map; map uses (x,y) not (row,col)
    test_map = np.zeros((300, 400))
    # Boundaries
    test_map[[0,-1],:] = 1
    test_map[:,[0,-1]] = 1
    # Horizontal walls
    test_map[0:125, [100,200,300]] = 1
    test_map[175:, [100,200,300]] = 1
    # Vertical walls
    test_map[[125,175], 75:125] = 1
    test_map[[125,175], 175:225] = 1
    test_map[[125,175], 275:325] = 1
    # Plot map
    xx,yy = np.meshgrid(np.arange(test_map.shape[0]), np.arange(test_map.shape[1]))
    mask = test_map[xx,yy] > 0.5
    plt.scatter(xx[mask],yy[mask])
    # plt.show()

    # Function inputs
    test_pnt = np.array([1,1])
    test_angle = 0
    num_vert = 10
    radius = 5
    max_range = 1000
    threshold = 0.5
    # Test
    get_line(test_pnt, np.pi/3, test_map, max_range, threshold)
    polygon_generator(test_pnt, test_angle, test_map, num_vert, radius, max_range, threshold)
    # Visualize results
    print("Visualization:")
