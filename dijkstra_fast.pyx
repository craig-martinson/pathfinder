#cython: language_level=3

from cython.operator import preincrement

import numpy as np
cimport numpy as np

ctypedef np.int_t DTYPE_t

def find_path_dijkstra(np.ndarray[DTYPE_t, ndim=2] grid, tuple start, tuple goal):
    # Convert the grid into a graph
    cdef dict graph = {}
    cdef int x, y, i
    cdef tuple neighbor, current
    for x in range(grid.shape[0]):
        for y in range(grid.shape[1]):
            if not grid[x, y]:
                neighbors = []
                if x > 0 and not grid[x - 1, y]:
                    neighbors.append((x - 1, y))
                if x < grid.shape[0] - 1 and not grid[x + 1, y]:
                    neighbors.append((x + 1, y))
                if y > 0 and not grid[x, y - 1]:
                    neighbors.append((x, y - 1))
                if y < grid.shape[1] - 1 and not grid[x, y + 1]:
                    neighbors.append((x, y + 1))
                graph[(x, y)] = neighbors

    # Use Dijkstra's algorithm to find the shortest path with only horizontal or vertical segments
    cdef dict dist = {start: 0}
    cdef dict prev = {start: None}
    cdef set visited = set([start])

    cdef int alt, i
    cdef tuple min_dist_node, neighbor, current
    cdef float min_dist

    while True:
        # Find the unvisited node with the smallest distance
        min_dist = 1.0/0.0  # positive infinity
        min_dist_node = None
        for node in dist:
            if node not in visited and dist[node] < min_dist:
                min_dist = dist[node]
                min_dist_node = node

        # If no unvisited node with a finite distance was found, terminate
        if min_dist_node is None:
            break

        current = min_dist_node
        visited.add(current)

        # Update the distances of the neighbors of the current node
        for neighbor in graph[current]:
            alt = (
                dist[current]
                + abs(current[0] - neighbor[0])
                + abs(current[1] - neighbor[1])
            )
            if neighbor not in dist or alt < dist[neighbor]:
                dist[neighbor] = alt
                prev[neighbor] = current

    # Build the optimized path by tracing back from the goal to the start
    cdef list path = []
    current = goal
    while current is not None:
        path.append(current)
        current = prev[current]
    path.reverse()

    # Remove unnecessary corners
    i = 0
    while i < len(path) - 2:
        if (path[i][0] == path[i + 1][0] and path[i + 1][0] == path[i + 2][0]) or (
            path[i][1] == path[i + 1][1] and path[i + 1][1] == path[i + 2][1]
        ):
            # Remove the middle point
            path.pop(i + 1)
        else:
            i += 1

    return path
