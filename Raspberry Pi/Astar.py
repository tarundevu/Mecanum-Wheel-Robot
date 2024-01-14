import math
import numpy as np

class Node():
    def __init__(self, parent=None,coord=None, obstacle=False):
        self.parent = parent
        self.coord = coord
        self.obstacle = obstacle
        self.g = 0  # Cost from start
        self.h = 0  # Heuristic (estimated cost to goal)
        self.f = 0  # Total cost: g + h
        
    def __eq__(self, other):
        return self.coord == other.coord

class Map:
    def __init__(self,w,h,node_dist):
        self.width_cm = w
        self.height_cm = h
        self.node_dist_cm = node_dist  # Distance between nodes in centimeters

        # Convert dimensions to number of nodes
        self.tile_width = self.width_cm // self.node_dist_cm
        self.tile_height = self.height_cm // self.node_dist_cm

        # Create an empty grid
        self.grid = np.zeros((self.tile_width, self.tile_height))
        self.obstacles = [(4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 6), (4, 5)]  # Set of obstacle nodes

    def addObstacles(self):
        for row in range(len(self.obstacles)):
            self.grid[self.obstacles[row][0],self.obstacles[row][1]] = 1
        return self.grid
    
    def grid_to_realworld(self,x,y):
        return x*self.node_dist_cm, y * self.node_dist_cm

def getHeuristic(node, goal, node_dist_cm):
    # odified heuristic to consider the distance between nodes
    dist = math.dist(node.coord, goal.coord) * node_dist_cm
    return dist


def getNeighbors(grid, node):
    neighbors = []
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]  # Right, Left, Down, Up, Diagonal

    for dx, dy in directions:
        x, y = node.coord[0] + dx, node.coord[1] + dy
        # Make sure within range
        if x > (len(grid) - 1) or x < 0 or y > (len(grid[len(grid)-1]) -1) or y < 0:
            continue

        # Make sure walkable terrain
        if grid[x][y] != 0:
            continue

        new_node = Node(node,(x,y))
        neighbors.append(new_node)

    return neighbors


def astar(grid, start, end):
    # initialize nodes
    start_node = Node(None,start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None,end)
    end_node.g = end_node.h = end_node.f = 0

    # initilaize open and closed lists
    open_list = []
    closed_list = []

    # add start node to list
    open_list.append(start_node)
    

    while len(open_list) > 0:
        # current node is start node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)
        

        # if goal found
        if current_node == end_node:
            path = []
            while current_node:
                path.append(current_node.coord)
                current_node = current_node.parent
            return path[::-1]  # Return reversed path
        
        neighbors = getNeighbors(grid,current_node)
        
        for neighbor in neighbors:
            if neighbor in closed_list or neighbor.obstacle:
                continue

            tentative_g = current_node.g + 1  # Assuming each step has a cost of 1
            neighbor.parent = current_node
            neighbor.g = tentative_g
            neighbor.h = getHeuristic(neighbor, end_node, 1)  
            neighbor.f = neighbor.g + neighbor.h

            if neighbor not in open_list:
                open_list.append(neighbor)
    

def print_path(grid, path):
    if path:
        for col in range(len(grid[0])):
            for row in range(len(grid)):
                if (col, row) in path:
                    print("* ", end="")
                elif grid[col][row] == 1:
                    print("X ", end="")
                else:
                    print(". ", end="")
            print()
    else:
        print("no path found")

def main():
    map = Map(10,10,1)
    map.addObstacles()
    grid = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 1, 0, 0, 0, 0]]

    start = (0, 0)
    end = (5, 6)

    path = astar(grid, start, end)
    print_path(grid,path)
    print(path)
    
    # print(map.grid)


if __name__ == '__main__':
    main()