from collections import defaultdict

def grid_to_graph(binary_grid, resolution):
    from math import sqrt
    graph = defaultdict(list)
    rows, cols = binary_grid.shape

    for i in range(rows):
        for j in range(cols):
            if binary_grid[i][j] == 1:
                for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                    ni, nj = i + dx, j + dy
                    if 0 <= ni < rows and 0 <= nj < cols and binary_grid[ni][nj] == 1:
                        distance = resolution  # horizontal/vertical move
                        graph[(i, j)].append(((ni, nj), distance))
    return graph
