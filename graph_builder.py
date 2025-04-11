from collections import defaultdict

def grid_to_graph(binary_grid, resolution):
    from math import sqrt
    graph = defaultdict(list)
    rows, cols = binary_grid.shape

    for i in range(rows):
        for j in range(cols):
            if binary_grid[i][j] == 1:
                for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), (-1,-1), (-1,1), (1,-1), (1,1)]: # eight squares (west, east, north south, sw, nw, se, ne)
                    ni, nj = i + dx, j + dy
                    if 0 <= ni < rows and 0 <= nj < cols and binary_grid[ni][nj] == 1:
                        if dx == 0 or dy == 0:
                            distance = resolution        
                        else:
                            distance = sqrt(2)*resolution
                        graph[(i, j)].append(((ni, nj), distance))
    return graph
