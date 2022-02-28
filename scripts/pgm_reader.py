import numpy as np

def read_pgm():
    """Return a raster of integers from a PGM as a list of lists."""
    # Open file
    pgmf = open('../maps/willow-full.pgm', 'rb')
    # Discard first line; P5
    (pgmf.readline())
    # Discard second line; resolution
    (pgmf.readline())
    # Discard third line; dimensions
    (pgmf.readline())
    # Discard fourth line; depth
    (pgmf.readline())
    # Hardcode values for map for now
    width = 584
    height = 526
    depth = 255

    map = np.zeros((width,height))
    raster = []
    for y in range(height):
        row = []
        for y in range(width):
            row.append(ord(pgmf.read(1)))
        raster.append(row)
    map = np.flip(np.asarray(raster).T,axis=1)/depth
    map = 1-map
    return map

if __name__ == "__main__":
    from matplotlib import pyplot as plt
    import task_generator
    import time
    import skimage.filters

    # Seed RNG
    np.random.seed(0)

    # # Test reading map
    test_map = read_pgm()
    # plt.imshow(test_map, cmap='Greys')
    # plt.show()
    
    # # Run task_generator
    taskgen = task_generator.TaskGenerator(test_map, threshold = 0.18, num_samples=1000)
    start = time.time()
    waypoints = taskgen.generate_tasks(20)
    end = time.time()
    elapsed_time = end-start
    print("Elapsed time: ", elapsed_time, "s")
    taskgen.visualize_circles(waypoints[:,0:2], waypoints[:,2])
    plt.show()
    # Export waypoints
    np.save("../outputs/willow-full",waypoints[:,0:2])
    # Fewer waypoints
    otsu_threshold = skimage.filters.threshold_otsu(waypoints[:,2])
    otsu_mask = waypoints[:,2] >= otsu_threshold
    waypoints_lean = waypoints[otsu_mask]
    taskgen.visualize_circles(waypoints_lean[:,0:2], waypoints_lean[:,2])
    plt.show()
    # Export waypoints_lean
    np.save("../outputs/willow-full_lean",waypoints_lean[:,0:2])


