def read_pgm():
    """Return a raster of integers from a PGM as a list of lists."""
    # Open file
    pgmf = open('localization_test.pgm', 'rb')
    # Discard first line; P5
    print(pgmf.readline())
    # Discard second line; resolution
    print(pgmf.readline())
    # Discard third line; dimensions
    print(pgmf.readline())
    # Discard fourth line; depth
    print(pgmf.readline())
    # (width, height) = [int(i) for i in pgmf.readline().split()]
    # depth = int(pgmf.readline())
    # assert depth <= 255
    width = 157
    height = 183
    depth = 255

    raster = []
    for y in range(height):
        row = []
        for y in range(width):
            row.append(ord(pgmf.read(1)))
        raster.append(row)
    return raster

if __name__ == "__main__":
    print(read_pgm())