import numpy as np
import pyclipper
import polygon_generator as polygen

if __name__ == "__main__":
    from matplotlib import pyplot as plt
    from scipy import ndimage

    """ Get polygons """
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

    # Arguments
    test_pnts = np.array([
        [50, 50],
        [150, 50],
        [250, 50],
        [50, 150],
        [150, 150],
        [250, 150],
        [50, 250],
        [150, 250],
        [250, 250],
        [50, 350],
        [150, 350],
        [250, 350]
    ])
    test_angle = 0
    num_vert = 10
    radius = 10
    max_range = 100
    threshold = 0.5

    polys = polygen.multi_polygon_generation(test_pnts, test_angle, test_map, num_vert, radius, max_range, threshold)
    
    polygen.plot_map(test_map, 'k')
    plt.scatter(test_pnts[:,0], test_pnts[:,1])
    polygen.plot_polygon_list(polys)
    plt.show()

    """ Union """
    # Set up subject list
    subj_list = []
    for poly in polys:
        subj_list.append(poly.tolist())
    
    # Compute union
    pc = pyclipper.Pyclipper()
    pc.AddPaths(subj_list, pyclipper.PT_SUBJECT, True)
    solution = pc.Execute(pyclipper.CT_UNION, pyclipper.PFT_NONZERO)

    poly_list2 = []
    for poly in solution:
        poly_list2.append(np.matrix(poly))
    polygen.plot_map(test_map, 'k')
    polygen.plot_polygon_list(poly_list2)
    plt.show()

    print("Done")
