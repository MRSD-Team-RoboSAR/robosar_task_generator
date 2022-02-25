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
        # Compute circle of given radius with given center
        # Returns list of points presenting circle
        print("WIP")

if __name__ == "__main__":
    maxcircle = MaxCircle(None)
    maxcircle.compute_circle(None, None)
