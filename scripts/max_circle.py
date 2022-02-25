import numpy as np
from matplotlib import pyplot as plt

class MaxCircle:
    """
    Finds maximum circle centered about point in given map
    """
    def __init__(self, map, threshold = 0.5, resolution=8):
        # Map is 2D matrix of occupancy grid map
        # Threshold determines whether space is occupied or not
        # Resolution is number of points used to represent circle at 1 cell length
        self._map = map
        self._threshold = threshold
        self._resolution = resolution
    
    def compute_circle(self, center, radius):
        # Compute circle of given radius with given center; all inputs and outputs in cells
        # center is 1x2 matrix in cells
        # radius is scalar given in cells
        # Returns Mx2 matrix (x,y) of points presenting circle, in cells
        circle_center = center.astype(int)
        circle_res = np.ceil(self._resolution * radius) # Make resolution scale with radius
        angle_rad = np.arange(0, circle_res)*(2*np.pi/circle_res)
        # Compute x coordinates of points
        x_coord = circle_center[0] + radius * np.sin(angle_rad)
        y_coord = circle_center[1] + radius * np.cos(angle_rad)
        tortn = np.hstack((np.expand_dims(x_coord, 1),np.expand_dims(y_coord, 1))).astype(int)
        return tortn
    
    def visualize(self, center, circle):
        # Visualize circle center and circumference
        plt.scatter(center[0], center[1])
        plt.scatter(circle[:,0], circle[:,1])

if __name__ == "__main__":
    maxcircle = MaxCircle(None)
    test_center = np.array([0,0])
    test_circle = maxcircle.compute_circle(test_center, 20)
    
    maxcircle.visualize(test_center, test_circle)
    plt.grid()
    plt.title("compute_circle test")
    plt.show()
