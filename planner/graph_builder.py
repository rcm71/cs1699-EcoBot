from collections import defaultdict
from math import sqrt

def is_safe(binary_grid, i, j, buffer=1):
    rows, cols = binary_grid.shape
    for di in range(-buffer, buffer + 1):
        for dj in range(-buffer, buffer + 1):
            ni, nj = i + di, j + dj
            if 0 <= ni < rows and 0 <= nj < cols:
                if binary_grid[ni][nj] != 0:
                    return False
            else:
                return False
    return True

def grid_to_graph(binary_grid, resolution=1.0, buffer=1):
    graph = defaultdict(list)
    rows, cols = binary_grid.shape

    for i in range(rows):
        for j in range(cols):
            if is_safe(binary_grid, i, j, buffer):
                node = (j, i)  # (x, y)

                for dx, dy in [(-1,0), (1,0), (0,-1), (0,1), 
                               (-1,-1), (-1,1), (1,-1), (1,1)]:
                    
                    ni, nj = i + dy, j + dx
                    if 0 <= ni < rows and 0 <= nj < cols and is_safe(binary_grid, ni, nj, buffer):
                        if dx != 0 and dy != 0:
                            if binary_grid[i + dy][j] != 0 or binary_grid[i][j + dx] != 0:
                                continue
                            dist = sqrt(2) * resolution
                        else:
                            dist = resolution

                        neighbor = (nj, ni)
                        graph[node].append((neighbor, dist))
    return graph
