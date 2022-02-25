import numpy as np
from matplotlib import pyplot as plt

import max_circle

class TaskGenerator:
    """
    Generates tasks for a given map
    """
    def __init__(self, map, num_tasks, threshold = 0.5, resolution=8):
        """
        map is 2D matrix of occupancy grid map in (x,y), not (row,col)
        num_tasks is number of tasks to allocate in map
        threshold determines whether space is occupied or not
        resolution is number of points used for collision detection
        """
        self._map = map
        self._num_tasks = num_tasks
        self._threshold = threshold
        self._maxcircle = max_circle.MaxCircle(map, threshold, resolution)
    
    def generate_random_points(self, num_pts):
        """
        Generates a number of random points in the map without collisions
        """
        x_coord = np.zeros((num_pts,)).astype(int)
        y_coord = np.zeros((num_pts,)).astype(int)
        collision_mask = np.ones((num_pts,)).astype(int)
        is_collision = True
        while(is_collision):
            x_coord[collision_mask] = np.random.randint(0, self._map.shape[0], np.count_nonzero(collision_mask))
            y_coord[collision_mask] = np.random.randint(0, self._map.shape[1], np.count_nonzero(collision_mask))
            map_vals = self._map[x_coord,y_coord]
            collision_mask = map_vals > self._threshold
            is_collision = np.any(collision_mask)
        pts = np.hstack((np.expand_dims(x_coord,1), np.expand_dims(y_coord,1)))
        return pts
    
    def max_circle_pts(self, pts):
        """
        Generates array of maximum radius for each given point
        """
        max_radius_array = np.zeros((pts.shape[0],))
        for i in range(0, pts.shape[0]):
            max_radius_array[i] = self._maxcircle.max_radius(pts[i])
        return max_radius_array
    
    def visualize(self, pts):
        """
        Visualize map and given points
        """
        # Plot map
        xx,yy = np.meshgrid(np.arange(self._map.shape[0]), np.arange(self._map.shape[1]))
        mask = test_map[xx,yy] > 0.5
        plt.scatter(xx[mask],yy[mask])
        # Plot pts
        plt.scatter(pts[:,0], pts[:,1])

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
    # plt.imshow(test_map.T, cmap="Greys")
    # plt.title("Test map")
    # plt.show()

    # # Constructor
    test_num_tasks = 8
    taskgen = TaskGenerator(test_map, test_num_tasks)
    
    # # Test generate_random_points
    test_pts = taskgen.generate_random_points(100)
    taskgen.visualize(test_pts)
    plt.title("generate_random_points")
    plt.show()

    # # Test max_circle_pts
    test_max_r_array = taskgen.max_circle_pts(test_pts)
    for i in range(0,100):
        taskgen._maxcircle.visualize(test_pts[i,:], taskgen._maxcircle.compute_circle(test_pts[i,:], test_max_r_array[i]))
    plt.show()