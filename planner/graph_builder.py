from collections import defaultdict
from math import sqrt

def grid_to_graph(binary_grid, resolution=1.0):
    graph = defaultdict(list)
    rows, cols = binary_grid.shape

    for i in range(rows):
        for j in range(cols):
            if binary_grid[i][j] == 0:  # walkable
                
                node = (j, i)
                
                for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), 
                               (-1,-1), (-1,1), (1,-1), (1,1)]:

                    ni, nj = i + dy, j + dx  # Apply offsets
                    if 0 <= ni < rows and 0 <= nj < cols and binary_grid[ni][nj] == 0:
                        if dx != 0 and dy != 0:
                            # For diagonal moves, check adjacent sides to prevent clipping
                            if binary_grid[i + dy][j] != 0 or binary_grid[i][j + dx] != 0:
                                continue  # Skip if either side is blocked
                            dist = sqrt(2) * resolution
                        else:
                            dist = resolution

                        neighbor = (nj, ni)  # (x,y) format
                        graph[node].append((neighbor, dist))
    return graph