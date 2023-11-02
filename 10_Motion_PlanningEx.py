import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import math
import heapq

# Define robot parameters
a1, a2 = 1, 1  # Link lengths

# Joint configuration
q = [np.pi/2, np.pi/1.5]


# Compute end-effector position using forward kinematics
x = a1 * np.cos(q[0]) + a2 * np.cos(q[0] + q[1])
y = a1 * np.sin(q[0]) + a2 * np.sin(q[0] + q[1])

# Create Obstacle
center = (0.9, 0.9)  # Change the coordinates as needed
radius = 0.6  # Change the radius as needed
circle = Circle(center, radius, fill=False, color='blue')

def forwardKinematics(q1, q2):
    x = a1 * np.cos(q1) + a2 * np.cos(q1 + q2)
    y = a1 * np.sin(q1) + a2 * np.sin(q1 + q2)
    return [x, y]
    
def checkObstacle(q1, q2, center, radius):
    pass

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = 0  # Cost from start to current node
        self.h = 0  # Heuristic (estimated cost from current node to goal)
        self.f = 0  # Total cost (f = g + h)
        self.parent = None
        self.passable = True  # Whether the cell is passable or blocked
    
    # Define the __lt__ method to enable comparisons between nodes
    def __lt__(self, other):
        return self.f < other.f

def heuristic(node, goal):
    # Manhattan distance heuristic
    return abs(node.x - goal.x) + abs(node.y - goal.y)

def get_neighbors(node, grid):
    neighbors = []
    x, y = node.x, node.y
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]  # Possible move directions: right, left, down, up

    for dx, dy in directions:
        new_x, new_y = x + dx, y + dy

        # Check if the neighbor is within the grid boundaries
        if 0 <= new_x < len(grid) and 0 <= new_y < len(grid[0]) and grid[new_x][new_y].passable:
            neighbors.append(grid[new_x][new_y])

    return neighbors

def astar(grid, start, goal):
    open_set = []
    closed_set = set()
    heapq.heappush(open_set, (start.f, start))

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(current)

        closed_set.add(current)

        for neighbor in get_neighbors(current, grid):
            if neighbor in closed_set:
                continue

            tentative_g = current.g + 1  # Assuming each step has a cost of 1

            if any(neighbor == (neighbor.f, neighbor) for _, neighbor in open_set) and tentative_g >= neighbor.g:
                continue

            neighbor.g = tentative_g
            neighbor.h = heuristic(neighbor, goal)
            neighbor.f = neighbor.g + neighbor.h
            neighbor.parent = current

            heapq.heappush(open_set, (neighbor.f, neighbor))

    return None  # No path found

def reconstruct_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]  # Reverse the path to start from the start node

resolution = 20
# Create a grid (example resolution x resolution grid)
grid_width = resolution
grid_height = resolution

# Initialize the grid with open cells
grid = [[Node(x, y) for y in range(grid_height)] for x in range(grid_width)]


fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6))

for q1 in range(resolution):
    for q2 in range(resolution):
        # Implement here
        
        result = checkObstacle(pose, q1_rad, q2_rad, center, radius)
        if result == True:
            ax2.scatter(q1 * scale, q2 * scale, color='red', marker='o')
            grid[q1][q2].passable = False
        else:
            ax2.scatter(q1 * scale, q2 * scale, color='blue', marker='o')    
            grid[q1][q2].passable = True
        pass
        


# Define the starting point and goal point
goal = grid[int(resolution/2)-1][int(resolution/2)-1]
start = grid[int(resolution/4)-1][int(resolution/4)-1]

sol_path = astar(grid, start, goal)

for i in range(len(sol_path)):
    scale = 360 / resolution
    q1 = sol_path[i][0] * scale * math.pi / 180
    q2 = sol_path[i][1] * scale * math.pi / 180
    pose = forwardKinematics(q1, q2)
    ax1.scatter( pose[0], pose[1], color='red', marker='x' )


ax2.set_aspect('equal')
plt.grid(True)
plt.title("")

ax2.scatter(sol_path[0][0] * (360 / resolution), sol_path[0][1] * (360 / resolution),color='red', marker='x')

for i in range(len(sol_path)):
    scale = 360 / resolution
    q1 = sol_path[i][0] * scale * math.pi / 180
    q2 = sol_path[i][1] * scale * math.pi / 180
    pose = forwardKinematics(q1, q2)
    ax1.clear()
    ax1.add_patch(circle)
    ax1.set_xlim([-a1-a2, a1+a2])
    ax1.set_ylim([-a1-a2, a1+a2])
    ax2.set_xlim([0, 360])
    ax2.set_ylim([0, 360])
    ax1.set_aspect('equal')

    ax1.plot([0, a1 * np.cos(q1), pose[0]], [0, a1 * np.sin(q1), pose[1]], 'o-')  # Plot robot
    
    scale = 360 / resolution
    ax2.scatter(sol_path[i][0] * scale, sol_path[i][1] * scale,color='green', marker='x')
    plt.pause(0.1)

plt.show()