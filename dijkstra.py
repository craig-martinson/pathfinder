from queue import PriorityQueue

def find_path_dijkstra(grid, start, goal):
    # Convert the grid into a graph
    graph = {}
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            if grid[x][y] == 0:
                neighbors = []
                if x > 0 and grid[x - 1][y] == 0:
                    neighbors.append((x - 1, y))
                if x < len(grid) - 1 and grid[x + 1][y] == 0:
                    neighbors.append((x + 1, y))
                if y > 0 and grid[x][y - 1] == 0:
                    neighbors.append((x, y - 1))
                if y < len(grid[0]) - 1 and grid[x][y + 1] == 0:
                    neighbors.append((x, y + 1))
                graph[(x, y)] = neighbors

    # Use Dijkstra's algorithm to find the shortest path with only horizontal or vertical segments
    dist = {start: 0}
    prev = {start: None}
    queue = PriorityQueue()
    queue.put((0, start))

    while not queue.empty():
        _, current = queue.get()
        if current == goal:
            break
        for neighbor in graph[current]:
            alt = (
                dist[current]
                + abs(current[0] - neighbor[0])
                + abs(current[1] - neighbor[1])
            )
            if neighbor not in dist or alt < dist[neighbor]:
                if neighbor in dist:
                    queue.put((alt, neighbor))
                else:
                    queue.put((alt, neighbor))
                    dist[neighbor] = alt
                    prev[neighbor] = current

    # Build the optimized path by tracing back from the goal to the start
    path = []
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