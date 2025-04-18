import heapq

def dijkstra(graph, start, end, return_cost=False):
    visited = set()
    pq = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}

    while pq:
        cost, node = heapq.heappop(pq)
        if node == end:
            break
        if node in visited:
            continue
        visited.add(node)
        for neighbor, move_cost in graph[node]:
            new_cost = cost_so_far[node] + move_cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                heapq.heappush(pq, (new_cost, neighbor))
                came_from[neighbor] = node

    path = []
    curr = end
    while curr != start:
        path.append(curr)
        curr = came_from.get(curr)
        if curr is None:
            return None if not return_cost else (None, float('inf'))
    path.append(start)
    path.reverse()

    return path if not return_cost else (path, cost_so_far[end])
