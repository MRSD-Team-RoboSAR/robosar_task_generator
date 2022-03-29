""" Based on following paper: Free-space Polygon Creation based on Occupancy Grid Maps for Trajectory Optimization Methods"""

import numpy as np
from matplotlib import pyplot as plt
import pyclipper
import polygon_generator as pg

if __name__ == "__main__":
    subj = (
    ((180, 200), (260, 200), (260, 150), (180, 150)),
    ((215, 160), (230, 190), (200, 190))
    )
    clip = ((190, 210), (240, 210), (240, 130), (190, 130))
    
    poly1 = np.matrix(subj[0])
    poly2 = np.matrix(subj[1])
    poly3 = np.matrix(clip)
    poly_list = []
    poly_list.append(poly1)
    poly_list.append(poly2)
    poly_list.append(poly3)
    pg.plot_polygon_list(poly_list)
    plt.show()

    pc = pyclipper.Pyclipper()
    pc.AddPath(clip, pyclipper.PT_CLIP, True)
    pc.AddPaths(subj, pyclipper.PT_SUBJECT, True)

    solution = pc.Execute(pyclipper.CT_INTERSECTION, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD)
    sol1 = np.matrix(solution[0])
    sol2 = np.matrix(solution[1])
    poly_list2 = []
    poly_list2.append(sol1)
    poly_list2.append(sol2)
    pg.plot_polygon_list(poly_list2)
    plt.show()
    print("Done")
