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
    
    def remove_overlaps(self, centers, radii):
        """
        Find and remove centers that are included in other circles
        Circle with larger radius have priority
        Returns centers and radii without overlap
        """
        to_delete = np.zeros((centers.shape[0],)).astype(bool)
        for i in range(0,centers.shape[0]):
            cur_center = centers[i]
            cur_radius = radii[i]
            other_centers = np.delete(np.copy(centers), i, axis=0)
            other_radii = np.delete(np.copy(radii), i)
            other_idx = np.delete(np.arange(0,centers.shape[0]), i)
            # Can eliminiate candidates that are further than radius in x or y directions
            candidates_mask_x = np.logical_and(other_centers[:,0] > (cur_center[0]-cur_radius), other_centers[:,0] < (cur_center[0]+cur_radius))
            candidates_mask_y = np.logical_and(other_centers[:,0] > (cur_center[0]-cur_radius), other_centers[:,0] < (cur_center[0]+cur_radius))
            candidates_mask = np.logical_and(candidates_mask_x, candidates_mask_y)
            candidates = other_centers[candidates_mask]
            candidates_radii = other_radii[candidates_mask]
            candidates_idx = other_idx[candidates_mask]
            # Check if candidates are within cur_circle
            candidate_dists = np.linalg.norm(cur_center-candidates,axis=1)
            confirmed_mask = candidate_dists < cur_radius
            confirmed = candidates[confirmed_mask]
            confirmed_radii = candidates_radii[confirmed_mask]
            confirmed_idx = candidates_idx[confirmed_mask]
            # Flag smaller or equal circles for deletion
            smaller_mask = confirmed_radii <= cur_radius
            to_delete[confirmed_idx[smaller_mask]] = True
        to_keep = np.logical_not(to_delete)
        new_centers = centers[to_keep]
        new_radii = radii[to_keep]
        return new_centers, new_radii
            
    
    def visualize_pts(self, pts):
        """
        Visualize map and given points
        """
        # Plot map
        xx,yy = np.meshgrid(np.arange(self._map.shape[0]), np.arange(self._map.shape[1]))
        mask = self._map[xx,yy] > 0.5
        plt.scatter(xx[mask],yy[mask])
        # Plot pts
        plt.scatter(pts[:,0], pts[:,1])
    
    def visualize_circles(self, centers, radii):
        # Plot map
        xx,yy = np.meshgrid(np.arange(self._map.shape[0]), np.arange(self._map.shape[1]))
        mask = self._map[xx,yy] > 0.5
        plt.scatter(xx[mask],yy[mask])
        # Plot circles
        for i in range(0, centers.shape[0]):
            center = centers[i,:]
            radius = radii[i]
            circle = self._maxcircle.compute_circle(center, radius)
            plt.scatter(center[0], center[1])
            plt.scatter(circle[:,0], circle[:,1])


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
    # taskgen.visualize_pts(test_pts)
    # plt.title("generate_random_points test")
    # plt.show()

    # # Test max_circle_pts
    test_max_r_array = taskgen.max_circle_pts(test_pts)
    # taskgen.visualize_circles(test_pts, test_max_r_array)
    # plt.title("max_cricle_pts test")
    # plt.show()

    # # Test remove_overlaps
    test_new_centers, test_new_radii = taskgen.remove_overlaps(test_pts, test_max_r_array)
    # taskgen.visualize_circles(test_new_centers, test_new_radii)
    # plt.title("remove_overlaps test")
    # plt.show()
    print('nop')