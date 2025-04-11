import heapq

def dijkstra(graph, start, end):
    visited = set()
    pq = [(0, start)]
    came_from = {}

    while pq:
        cost, node = heapq.heappop(pq)
        if node == end:
            break
        if node in visited:
            continue
        visited.add(node)
        for neighbor in graph[node]:
            if neighbor not in visited:
                heapq.heappush(pq, (cost + 1, neighbor))
                if neighbor not in came_from:
                    came_from[neighbor] = node

    path = []
    curr = end
    while curr != start:
        path.append(curr)
        curr = came_from.get(curr)
        if curr is None:
            return None
    path.append(start)
    path.reverse()
    return path
