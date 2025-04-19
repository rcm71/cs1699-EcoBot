from collections import defaultdict
from math import sqrt

def grid_to_graph(binary_grid, resolution=1.0):
    from collections import defaultdict
    from math import sqrt

    graph = defaultdict(list)
    rows, cols = binary_grid.shape

    for i in range(rows):
        for j in range(cols):
            if binary_grid[i][j] == 0:  # walkable
                for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), 
                               (-1,-1), (-1,1), (1,-1), (1,1)]:

                    ni, nj = i + dx, j + dy
                    if 0 <= ni < rows and 0 <= nj < cols and binary_grid[ni][nj] == 0:
                        if dx != 0 and dy != 0:
                            # for diagonal moves, check adjacent sides to prevent clipping! 
                            if binary_grid[i + dx][j] != 0 or binary_grid[i][j + dy] != 0:
                                continue  # skip if either side is blocked
                            dist = sqrt(2) * resolution
                        else:
                            dist = resolution

                        graph[(i, j)].append(((ni, nj), dist))
    return graph

