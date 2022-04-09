import numpy as np
from matplotlib import pyplot as plt
from scipy import ndimage

# kernel = np.array([
#     [128, 1, 2],
#     [64, 0, 4],
#     [32, 16, 8]
# ])

def get_boundary_pixels(fa_map):
    """
    Takes in Free Area Map and produces three nx2 matrix for boundary pixels, edge pixels, and corner pixels
    For FAM, free areas (foreground pixels) should have value 1; occupied areas (background pixels) should have value 0
    """
    kernel = np.array([
        [128, 1, 2],
        [64, -256, 4],
        [32, 16, 8]
    ])
    # Make foreground pixels have value of 0, background pixels have vlue of 1
    neg_map = np.logical_not(fa_map)*1
    # Compute weight for each pixel; background pixels will have weight of 0
    neg_map_weight = ndimage.correlate(neg_map, kernel)
    neg_map_weight[neg_map_weight<0]=0
    # Boundary pixels have weights greater than 0
    boundary_mask = neg_map_weight > 0
    # Edge pixels have weights greater than 0 after ignoring corner neightbors
    ignore_corners_mask = 0b01010101
    edge_mask = np.bitwise_and(neg_map_weight, ignore_corners_mask)>0
    # If boundary pixel is not an edge pixel, then it is a corner pixel
    corner_mask = np.logical_and(boundary_mask,np.logical_not(edge_mask))
    # Get coordinates
    boundary_pixels = np.zeros((1,2),dtype='int')
    edge_pixels = np.zeros((1,2),dtype='int')
    corner_pixels = np.zeros((1,2),dtype='int')
    for x in range(0,fa_map.shape[0]):
        for y in range(0,fa_map.shape[1]):
            if(boundary_mask[x,y]):
                cur_coord = np.array([x,y]).reshape(1,2)
                # Is boundary pixel
                boundary_pixels = np.vstack((boundary_pixels, cur_coord))
                if(edge_mask[x,y]):
                    # Is edge pixel
                    edge_pixels = np.vstack((edge_pixels, cur_coord))
                else:
                    # Is corner pixel
                    corner_pixels = np.vstack((corner_pixels, cur_coord))
    boundary_pixels = boundary_pixels[1:]
    edge_pixels = edge_pixels[1:]
    corner_pixels = corner_pixels[1:]
    return boundary_pixels, edge_pixels, corner_pixels

def check_neighbours(fa_map, coords):
    """
    Takes in Free Area Map and produces list of points in coords (nx2) that have 2, 3, 4 contiguous neighbours; also list of non-skeletal pixels
    For FAM, free areas (foreground pixels) should have value 1; occupied areas (background pixels) should have value 0
    """
    kernel = np.array([
        [128, 1, 2],
        [64, 0, 4],
        [32, 16, 8]
    ])
    S2_weights = [3, 6, 12, 24, 48, 96, 192, 129]
    S3_weights = [7, 14, 28, 56, 112, 224, 193, 131]
    S4_weights = [15, 30, 60, 120, 240, 225, 195, 135]
    Sns_weights = [3, 5, 7, 12, 13, 14, 15, 20, 21, 22, 23, 28, 29, 30, 31, 48,
        52, 53, 54, 55, 56, 60, 61, 62, 63, 65, 67, 69, 71, 77, 79,
        80, 81, 83, 84, 85, 86, 87, 88, 89, 91, 92, 93, 94, 95, 97,
        99, 101, 103, 109, 111, 112, 113, 115, 116, 117, 118,
        119, 120, 121, 123, 124, 125, 126, 127, 131, 133, 135,
        141, 143, 149, 151, 157, 159, 181, 183, 189, 191, 192,
        193, 195, 197, 199, 205, 207, 208, 209, 211, 212, 213,
        214, 215, 216, 217, 219, 220, 221, 222, 223, 224, 225,
        227, 229, 231, 237, 239, 240, 241, 243, 244, 245, 246,
        247, 248, 249, 251, 252, 253, 254, 255]
    S2 = []
    S3 = []
    S4 = []
    Sns = []
    fa_map_weight = ndimage.correlate(fa_map, kernel)
    for x,y in coords:
        cur_weight = fa_map_weight[x,y]
        # Check if in S2, S3, S4 or Sns
        if(S2_weights.count(cur_weight)):
            S2.append([x,y])
        elif(S3_weights.count(cur_weight)):
            S3.append([x,y])
        elif(S4_weights.count(cur_weight)):
            S4.append([x,y])
        elif(Sns_weights.count(cur_weight)):
            Sns.append([x,y])
    S2 = np.array(S2)
    S3 = np.array(S3)
    S4 = np.array(S4)
    Sns = np.array(Sns)
    return S2, S3, S4, Sns




def plot_map(map, color = None, threshold=0.5):
    # Visualize map
    xx,yy = np.meshgrid(np.arange(map.shape[0]), np.arange(map.shape[1]))
    mask = map[xx,yy] > threshold
    plt.scatter(xx[mask],yy[mask], color = color, s=0.1)

def image_thinning(fa_map):
    """
    Creates RAGVD from Free Area Map
    Free Area Map is binary; consists only of 0 and 1
    """
    ogm = np.logical_not(fa_map)*1
    fa_map_copy = np.copy(fa_map)
    isdeleted = True
    i = 0
    while(isdeleted):
        i += 1
        print("iter: ", i)
        isdeleted = False
        # Compute boundary, edge and corner pixels
        boundary_pixels, edge_pixels, corner_pixels = get_boundary_pixels(fa_map_copy)
        print("Num boundary points: ", boundary_pixels.shape[0])
        # Check against S2, S3, S4, Sns
        S2, S3, S4, Sns = check_neighbours(fa_map_copy, boundary_pixels)
        # Combine sets
        S = np.vstack((S2.reshape((-1,2)), S3.reshape((-1,2)), S4.reshape((-1,2)), Sns.reshape((-1,2)))).astype('int')
        # S = Sns
        # S = Sns
        # plot_map(ogm,'k')
        # plot_map(fa_map_copy,'r')
        # if(S.shape[0]>0):
        #     plt.scatter(S[:,0],S[:,1],color='g')
        # plt.show()
        print("Num points to zero: ", S.shape[0])
        if(S.shape[0]):
            isdeleted = True
            fa_map_copy[S[:,0],S[:,1]] = 0
    plot_map(ogm,'k')
    plot_map(fa_map_copy,'r')
    plt.show()

def read_pgm(file):
    """Return a raster of integers from a PGM as a list of lists."""
    # Open file
    pgmf = open(file, 'rb')
    # Discard first line; P5
    print(pgmf.readline())
    # Discard second line; resolution
    print(pgmf.readline())
    # Discard third line; dimensions
    print(pgmf.readline())
    # Discard fourth line; depth
    print(pgmf.readline())
    # Hardcode values for map for now
    width = 584
    height = 526
    depth = 255
    # Read in map from file
    map = np.zeros((width,height))
    raster = []
    for y in range(height):
        row = []
        for y in range(width):
            row.append(ord(pgmf.read(1)))
        raster.append(row)
    # Convert (row,col) to (x,y)
    map = np.flip(np.asarray(raster).T,axis=1)/depth
    # Make freespace = 0, make occupied = 1
    map = 1-map
    map = (map>0.1)*1
    return map

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
    
    # # Smaller test map
    # test_map = np.zeros((100,50))
    # # Boundaries
    # test_map[[0,-1],:] = 1
    # test_map[:,[0,-1]] = 1
    # # Horizontal walls
    # test_map[0:50, [15,35]] = 1
    # test_map[80:, [15,35]] = 1
    # # Thicken
    # test_map = ndimage.binary_dilation(test_map, iterations=3)
    # test_map = test_map.astype('int')

    # Test PGM
    # file = '/home/jsonglaptop/catkin_ws/src/robosar_navigation/maps/scott_hall_PR4.pgm'
    file = '/home/jsonglaptop/catkin_ws/src/robosar_navigation/maps/willow-full.pgm'
    test_map = read_pgm(file)
    # Plot
    plot_map(test_map)
    plt.show()

    """ Preprocessing """
    # Close operation
    # test_map = ndimage.morphology.binary_closing(test_map, iterations=10).astype(np.float64) # scott_hall_PR4
    test_map = ndimage.morphology.binary_closing(test_map, iterations=3).astype(np.float64)
    plot_map(test_map)
    plt.show()

    # Smoothing
    # test_map = (ndimage.gaussian_filter(test_map,2)>0.1).astype('int')
    test_map = (ndimage.gaussian_filter(test_map,1)>0.1).astype('int')
    plot_map(test_map)
    plt.show()

    # Turn OGM into FAM
    test_map = np.logical_not(test_map)*1
    # Plot map

    # Test
    # check_neighbours(test_map, coords)
    # get_boundary_pixels(test_map)
    image_thinning(test_map)
