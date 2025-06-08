from heapq import heappush, heappop

def a_star(grid, start, goal):
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: manhattan_distance(start, goal)}

    while open_set:
        current = heappop(open_set)[1]
        if current == goal:
            return reconstruct_path(came_from, current)
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if grid.is_valid_move(*neighbor):
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + manhattan_distance(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))
    return []

def manhattan_distance(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]

def multi_a_star(grid, start_positions, goals):
    paths = []
    for start, goal in zip(start_positions, goals):
        path = a_star(grid, start, goal)
        paths.append(path)
    return paths