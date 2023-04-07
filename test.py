import pygame
import heapq
import random
import math

# Define the constants
GRID_WIDTH = 200
GRID_HEIGHT = 200
CELL_WIDTH = 5
CELL_HEIGHT = 5

# Define the colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)


def random_empty_point(grid):
    empty_points = []
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if grid[i][j] == 0:
                empty_points.append((i, j))
    if len(empty_points) == 0:
        return None
    else:
        return random.choice(empty_points)


def generate_rectangles(
    x,
    y,
    width,
    height,
    *,
    max_rectangles=64,
    alignment_grid=10,
    min_width_multiple=1,
    max_width_multiple=2,
    min_height_multiple=1,
    max_height_multiple=2,
    border_multiple=1
):
    rectangles = []

    # Generate the rectangles
    for i in range(max_rectangles):
        rect_x = random.randint(0, width - 1)
        rect_y = random.randint(0, height - 1)

        # Align rectangle origin to coarse grid, realtive to parent origin
        rect_x = x + (rect_x + (alignment_grid // 2)) // alignment_grid * alignment_grid
        rect_y = y + (rect_y + (alignment_grid // 2)) // alignment_grid * alignment_grid

        # Generate rectangle
        w = random.randint(min_width_multiple, max_width_multiple) * alignment_grid
        h = random.randint(min_height_multiple, max_height_multiple) * alignment_grid

        # Ignore overlaping rects or rects that extend pass the border
        rect = [rect_x, rect_y, w, h]
        border = alignment_grid * border_multiple
        if is_rect_inside(
            rect, [x + border, y + border, width - 2 * border, height - 2 * border]
        ):
            if not overlaps(rect, rectangles) and not touches(rect, rectangles):
                rectangles.append(rect)

    return rectangles


def overlaps(rect1, rectangles):
    for rect2 in rectangles:
        if (
            rect1[0] < rect2[0] + rect2[2]
            and rect1[0] + rect1[2] > rect2[0]
            and rect1[1] < rect2[1] + rect2[3]
            and rect1[1] + rect1[3] > rect2[1]
        ):
            return True
    return False


def touches(rect1, rectangles):
    for rect2 in rectangles:
        if (
            rect1[0] < rect2[0] + rect2[2] + 1
            and rect1[0] + rect1[2] > rect2[0] - 1
            and rect1[1] < rect2[1] + rect2[3] + 1
            and rect1[1] + rect1[3] > rect2[1] - 1
        ):
            return True
    return False


def is_rect_inside(rect1, rect2):
    return (
        rect1[0] >= rect2[0]
        and rect1[0] + rect1[2] <= rect2[0] + rect2[2]
        and rect1[1] >= rect2[1]
        and rect1[1] + rect1[3] <= rect2[1] + rect2[3]
    )


# Define the function to rasterize rectangles in an array
def rasterize_rectangles(rectangles, grid):
    for rect in rectangles:
        x, y, w, h = rect
        for i in range(y, y + h):
            for j in range(x, x + w):
                if i >= 0 and i < len(grid) and j >= 0 and j < len(grid[0]):
                    grid[i][j] = 1
    return grid


def print_grid(grid):
    for row in grid:
        print(" ".join(str(cell) for cell in row))


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


def remove_corners(path, grid):
    if len(path) < 3:
        return path
    new_path = [path[0]]
    for i in range(2, len(path)):
        x1, y1 = path[i - 2]
        x2, y2 = path[i - 1]
        x3, y3 = path[i]
        if (x1 == x3 and y1 != y3) or (x1 != x3 and y1 == y3):
            # The cells are aligned, no corner
            new_path.append(path[i - 1])
        elif grid[x2][y3] == 0:
            # The cells are not aligned, but there is no obstacle
            new_path.append(path[i - 1])
    new_path.append(path[-1])
    return new_path


def minimize_corners_90(path):
    """
    Given a path generated by A-star algorithm, returns a modified path that minimizes the number of corners while using only horizontal or vertical segments.
    """
    # If the path has less than three points, there is no corner to eliminate.
    if len(path) < 3:
        return path

    # Initialize the new path with the first two points from the original path.
    new_path = [path[0], path[1]]

    # Iterate over the remaining points in the path.
    for i in range(2, len(path)):
        # Check if the current point forms a straight line with the previous two points.
        if (new_path[-1][0] - new_path[-2][0]) * (path[i][1] - new_path[-2][1]) == (
            new_path[-1][1] - new_path[-2][1]
        ) * (path[i][0] - new_path[-2][0]):
            # If it does, skip adding the current point to the new path.
            continue
        # Otherwise, check if a 90 degree turn is possible.
        if new_path[-1][0] == new_path[-2][0]:
            if path[i][0] == new_path[-1][0]:
                # If the current point is on the same vertical line, skip adding the intermediate point.
                continue
            else:
                # Add an intermediate point to force a horizontal turn.
                int_point = (path[i][0], new_path[-1][1])
        elif new_path[-1][1] == new_path[-2][1]:
            if path[i][1] == new_path[-1][1]:
                # If the current point is on the same horizontal line, skip adding the intermediate point.
                continue
            else:
                # Add an intermediate point to force a vertical turn.
                int_point = (new_path[-1][0], path[i][1])
        else:
            # This should not happen if the path is valid.
            raise ValueError("Invalid path")

        # Add the intermediate point and the current point to the new path.
        new_path.append(int_point)
        new_path.append(path[i])

    return new_path


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


def jump_search(start, end, grid):
    n = len(grid)
    m = len(grid[0])
    block_size = int(math.sqrt(n * m))

    x, y = start
    while True:
        if grid[x][y] != 0:
            return None

        if (x, y) == end:
            return [(x, y)]

        bx, by = x // block_size, y // block_size
        nbx, nby = (bx + 1) * block_size, (by + 1) * block_size

        if (
            end[0] >= bx * block_size
            and end[0] < nbx
            and end[1] >= by * block_size
            and end[1] < nby
        ):
            path = [(x, y)]
            dx, dy = end[0] - x, end[1] - y
            sx, sy = dx // abs(dx) if dx != 0 else 0, dy // abs(dy) if dy != 0 else 0
            while (x, y) != end:
                x += sx
                y += sy
                if grid[x][y] != 0:
                    return None
                path.append((x, y))
            return path

        if end[0] < bx * block_size:
            nx, ny = bx * block_size, by * block_size + block_size // 2
        elif end[0] >= nbx:
            nx, ny = nbx - 1, by * block_size + block_size // 2
        elif end[1] < by * block_size:
            nx, ny = bx * block_size + block_size // 2, by * block_size
        else:
            nx, ny = bx * block_size + block_size // 2, nby - 1

        dx, dy = end[0] - nx, end[1] - ny
        sx, sy = dx // abs(dx) if dx != 0 else 0, dy // abs(dy) if dy != 0 else 0
        if dx == dy == 0:
            return [(x, y), end]
        elif dx != 0 and dy != 0 and grid[nx + sx][ny] == 0 and grid[nx][ny + sy] == 0:
            x, y = nx + sx, ny + sy
        else:
            x, y = nx, ny


def print_path(path):
    if path:
        for row in path:
            print(" ".join(str(cell) for cell in row))
    else:
        print("Empty path")


# Initialize pygame
pygame.init()

# Set the window size and title
size = (GRID_WIDTH * CELL_WIDTH, GRID_HEIGHT * CELL_HEIGHT)
screen = pygame.display.set_mode(size)
pygame.display.set_caption("Pathfinding Tests")

# Generate the grid
grid = [[0 for col in range(GRID_WIDTH)] for row in range(GRID_HEIGHT)]

# Generate parent rectangles
parent_rects = generate_rectangles(
    0,
    0,
    GRID_WIDTH,
    GRID_HEIGHT,
    max_rectangles=GRID_WIDTH * GRID_HEIGHT // 2,
    alignment_grid=8,
    min_width_multiple=4,
    max_width_multiple=8,
    min_height_multiple=4,
    max_height_multiple=8,
)

# Generate the child rectangles
all_rectangles = []

for rect in parent_rects:
    x, y, w, h = rect

    rects = generate_rectangles(
        x,
        y,
        w,
        h,
        max_rectangles=GRID_WIDTH * GRID_HEIGHT // 2,
        alignment_grid=2,
        min_width_multiple=2,
        max_width_multiple=4,
        min_height_multiple=2,
        max_height_multiple=4,
    )

    # Keep track of all rects for rendering
    for rect in rects:
        all_rectangles.append(rect)

    rasterize_rectangles(rects, grid)

# Generate the initial path
start = random_empty_point(grid)
end = random_empty_point(grid)

path_a = find_path_astar(start, end, grid)
path_b = find_path_dijkstra(grid, start, end)

# Define the main program loop
done = False
clock = pygame.time.Clock()


def draw_path(cell_width, cell_height, path, screen, color, pan_x, pan_y, zoom):
    if path:
        for i in range(len(path) - 1):
            row1, col1 = path[i]
            row2, col2 = path[i + 1]

            pygame.draw.line(
                screen,
                color,
                [
                    pan_x + (col1 * cell_width + cell_width // 2) * zoom,
                    pan_y + (row1 * cell_height + cell_height // 2) * zoom,
                ],
                [
                    pan_x + (col2 * cell_width + cell_width // 2) * zoom,
                    pan_y + (row2 * cell_height + cell_height // 2) * zoom,
                ],
                2,
            )


def draw_rect(cell_width, cell_height, color, screen, rect, pan_x, pan_y, zoom):
    pygame.draw.rect(
        screen,
        color,
        [
            pan_x + (rect[0] * cell_width) * zoom,
            pan_y + (rect[1] * cell_height) * zoom,
            rect[2] * zoom * cell_width,
            rect[3] * zoom * cell_height,
        ],
    )


def draw_circle(cell_width, cell_height, color, screen, start, pan_x, pan_y, zoom):
    pygame.draw.circle(
        screen,
        color,
        [
            pan_x + (start[1] * cell_width + cell_width // 2) * zoom,
            pan_y + (start[0] * cell_height + cell_height // 2) * zoom,
        ],
        max(1, cell_width * zoom),
    )


# Set up the initial zoom and pan values
zoom = 1.0
offset = [0, 0]

while not done:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                # Generate a new path
                start = random_empty_point(grid)
                end = random_empty_point(grid)
                path_a = find_path_astar(start, end, grid)
                path_b = find_path_dijkstra(grid, start, end)
            elif event.key == pygame.K_ESCAPE:
                # Quit
                done = True
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 4:  # scroll up
                zoom *= 1.1
                # Calculate new offset based on mouse position
                mouse_pos = pygame.mouse.get_pos()
                offset[0] = mouse_pos[0] - (mouse_pos[0] - offset[0]) * 1.1
                offset[1] = mouse_pos[1] - (mouse_pos[1] - offset[1]) * 1.1
            elif event.button == 5:  # scroll down
                zoom /= 1.1
                # Calculate new offset based on mouse position
                mouse_pos = pygame.mouse.get_pos()
                offset[0] = mouse_pos[0] - (mouse_pos[0] - offset[0]) / 1.1
                offset[1] = mouse_pos[1] - (mouse_pos[1] - offset[1]) / 1.1
        elif event.type == pygame.MOUSEMOTION:
            if event.buttons[0]:  # Left button is held down
                offset[0] += event.rel[0]
                offset[1] += event.rel[1]

    # Clear the screen
    screen.fill(WHITE)

    # Draw the parent rectangles
    for rect in parent_rects:
        draw_rect(
            CELL_WIDTH, CELL_HEIGHT, GREEN, screen, rect, offset[0], offset[1], zoom
        )

    # Draw the rectangles
    for rect in all_rectangles:
        draw_rect(
            CELL_WIDTH, CELL_HEIGHT, BLACK, screen, rect, offset[0], offset[1], zoom
        )

    # Draw the path
    if len(path_a) < len(path_b):
        draw_path(
            CELL_WIDTH, CELL_HEIGHT, path_a, screen, BLUE, offset[0], offset[1], zoom
        )
    else:
        draw_path(
            CELL_WIDTH, CELL_HEIGHT, path_b, screen, PURPLE, offset[0], offset[1], zoom
        )

    # Draw the start and end points
    draw_circle(
        CELL_WIDTH, CELL_HEIGHT, ORANGE, screen, start, offset[0], offset[1], zoom
    )
    draw_circle(CELL_WIDTH, CELL_HEIGHT, RED, screen, end, offset[0], offset[1], zoom)

    # Update the screen
    pygame.display.flip()

    # Delay to achieve desired frame rate
    clock.tick(60)

# Quit pygame
pygame.quit()
