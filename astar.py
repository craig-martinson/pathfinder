
def neighbors(cell, grid):
    row, col = cell
    width, height = len(grid[0]), len(grid)

    candidates = [(row - 1, col), (row, col + 1), (row + 1, col), (row, col - 1)]
    result = []

    for r, c in candidates:
        if 0 <= r < height and 0 <= c < width and grid[r][c] != 1:
            result.append((r, c))

    return result


# Define the A* algorithm
def reconstruct_path(came_from, start, end):
    path = [end]
    current = end
    while current != start:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def heuristic(cell, goal):
    x_distance = abs(cell[0] - goal[0])
    y_distance = abs(cell[1] - goal[1])
    return x_distance + y_distance


def heuristic2(cell, goal):
    x_distance = abs(cell[0] - goal[0])
    y_distance = abs(cell[1] - goal[1])
    return x_distance + y_distance + min(x_distance, y_distance)


def cost(current, neighbor):
    if current[0] == neighbor[0] or current[1] == neighbor[1]:
        return 1  # Horizontal or vertical move
    else:
        return 1.41421356  # Diagonal move, approx. sqrt(2)


def find_path_astar(start, end, grid):
    open_set = [start]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}

    while open_set:
        current = min(open_set, key=lambda cell: f_score[cell])
        open_set.remove(current)

        if current == end:
            path = reconstruct_path(came_from, start, end)
            return path

        for neighbor in neighbors(current, grid):
            tentative_g_score = g_score[current] + cost(current, neighbor)

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)

                if neighbor not in open_set:
                    open_set.append(neighbor)

    return None