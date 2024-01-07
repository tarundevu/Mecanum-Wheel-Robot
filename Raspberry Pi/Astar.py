import heapq
import numpy as np
import math

class Node:
    def __init__(self, x, y, obstacle=False):
        self.x = x
        self.y = y
        self.obstacle = obstacle
        self.g = float('inf')  # Cost from start
        self.h = float('inf')  # Heuristic (estimated cost to goal)
        self.f = float('inf')  # Total cost: g + h
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f


class Map:
    def __init__(self):
        self.width_cm = 50
        self.height_cm = 50
        self.node_dist_cm = 2  # Distance between nodes in centimeters

        # Convert dimensions to number of nodes
        self.width_nodes = self.width_cm // self.node_dist_cm
        self.height_nodes = self.height_cm // self.node_dist_cm

        # Create an empty grid
        self.grid = np.zeros((self.width_nodes, self.height_nodes))
        self.obstacles = {(4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 6), (4, 5)}  # Set of obstacle nodes

    def create_grid(self):
        self.grid = [
            [Node(x, y, obstacle=(x, y) in self.obstacles) for y in range(self.height_nodes)] 
            for x in range(self.width_nodes)
        ]
        return self.grid
    def grid_to_realworld(self,x,y):
        return x*self.node_dist_cm,y * self.node_dist_cm

def heuristic(node, goal, node_dist_cm):
    # Modified heuristic to consider the distance between nodes
    dx = abs(node.x - goal.x) * node_dist_cm
    dy = abs(node.y - goal.y) * node_dist_cm
    return dx + dy


def astar(grid, start, goal, node_dist_cm):
    open_set = []
    closed_set = set()

    heapq.heappush(open_set, start)
    start.g = 0
    start.h = heuristic(start, goal, node_dist_cm)
    start.f = start.g + start.h

    while open_set:
        current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]

        closed_set.add(current)

        for neighbor in neighbors(grid, current):
            if neighbor in closed_set or neighbor.obstacle:
                continue

            tentative_g = current.g + 1  # Assuming each step has a cost of 1

            if tentative_g < neighbor.g:
                neighbor.parent = current
                neighbor.g = tentative_g
                neighbor.h = heuristic(neighbor, goal, node_dist_cm)
                neighbor.f = neighbor.g + neighbor.h

                if neighbor not in open_set:
                    heapq.heappush(open_set, neighbor)

    return None  # No path found


def neighbors(grid, node):
    neighbors = []
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]  # Right, Left, Down, Up, Diagonal

    for dx, dy in directions:
        x, y = node.x + dx, node.y + dy
        if 0 <= x < len(grid) and 0 <= y < len(grid[0]):
            neighbors.append(grid[x][y])

    return neighbors


def print_path(grid, path):
    for row in range(len(grid[0])):
        for col in range(len(grid)):
            if (col, row) in path:
                print("* ", end="")
            elif grid[col][row].obstacle:
                print("X ", end="")
            else:
                print(". ", end="")
        print()
    for coord in path:
        real_coord = map_instance.grid_to_realworld(coord[0],coord[1])
        print(real_coord, end=" ")


if __name__ == "__main__":
    # Example usage
    map_instance = Map()
    grid = map_instance.create_grid()
    start_node = Node(0, 0)
    goal_node = Node(math.floor(10//map_instance.node_dist_cm-1),math.floor(50//map_instance.node_dist_cm-1))  # Adjust the goal coordinates based on the grid size
    
    
    

    path = astar(grid, grid[start_node.x][start_node.y], grid[goal_node.x][goal_node.y], map_instance.node_dist_cm)

    if path:
        print("Path found:")
        print_path(grid, path)
    else:
        print("Invalid")