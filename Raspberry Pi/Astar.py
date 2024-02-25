import math
import numpy as np
from numpy import size
import time
import tkinter as tk

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
    def __init__(self,h,w,node_dist):
        self.width_cm = w
        self.height_cm = h
        self.node_dist_cm = node_dist  # Distance between nodes in centimeters

        # Convert dimensions to number of nodes
        self.tile_width = self.width_cm // self.node_dist_cm
        self.tile_height = self.height_cm // self.node_dist_cm

        # Create an empty grid
        self.grid = np.zeros((self.tile_height, self.tile_width))
        self.obstacles = set()  # Set of obstacle nodes
        self.expanded = set()
    def getGrid(self):
        return self.grid
    
    def addObstacles(self):
        grid = self.grid
        for x, y in self.obstacles.union(self.expanded):
            grid[x, y] = 1
        
        
    def createObs(self, x, y, w, h):
        x,y = x//self.node_dist_cm,y//self.node_dist_cm
        w, h = w//self.node_dist_cm,h//self.node_dist_cm
        obs = self.obstacles
        grid = self.grid
        # newx,newy = int(x-math.ceil(h*0.5)), int(y-math.ceil(w*0.5))
        
        for i in range(-round(h/2), round(h/2)):
            for j in range(-round(w/2), round(w/2)):
                newx, newy = x + i, y + j
                if 0 <= newx < size(grid, 0) and 0 <= newy < size(grid, 1):
                    if (newx,newy) not in self.obstacles:
                        obs.add((newx, newy))
                else:
                    print("Coordinates out of bounds")

    def createObsRect(self, x1, y1, x2, y2):
        x1, x2, y1, y2 = x1//self.node_dist_cm, x2 //self.node_dist_cm, y1 // self.node_dist_cm, y2 // self.node_dist_cm
        min_x, max_x, min_y, max_y = min(x1, x2), max(x1, x2), min(y1, y2), max(y1, y2)
         
        for x in range(min_x, max_x):
            for y in range(min_y, max_y):
                self.obstacles.add((x, y))

    def expandObs(self,dist):
        newobs = set()
        dist = dist // self.node_dist_cm
        for x, y in self.obstacles:
            for i in range(-dist, dist + 1):
                for j in range(-dist, dist + 1):
                    newx, newy = x + i, y + j
                    if 0 <= newx < size(self.grid, 0) and 0 <= newy < size(self.grid, 1) and (newx, newy) not in self.obstacles:
                        newobs.add((newx, newy))
        self.expanded.update(newobs)
        
        
                    
    def grid_to_realworld(self,x,y):
        return x*self.node_dist_cm, y * self.node_dist_cm
class AStarGridMapGUI:
    def __init__(self, root, map, path):
        self.root = root
        self.map = map
        self.path = path
        self.canvas = tk.Canvas(root, width=900, height=900, bg='white')
        self.canvas.pack()

    def draw_grid_map(self):
        num_rows, num_cols = self.map.grid.shape
        grid = self.map.grid
        exp = self.map.expanded
        create = self.canvas.create_rectangle
        cell_width = 600 // num_cols+1
        cell_height = 600 // num_rows+1
        
        for i in range(num_rows):
            for j in range(num_cols):
                x0, y0 = j * cell_width, i * cell_height
                x1, y1 = x0 + cell_width, y0 + cell_height
                if (i, j) in self.path:
                    create(x0, y0, x1, y1, fill='green', outline='green')
                elif grid[i][j] == 1:
                    if (i,j) in exp:
                        create(x0, y0, x1, y1, fill='red', outline='red')
                    else:
                        create(x0, y0, x1, y1, fill='black', outline='black')
                else:
                    create(x0, y0, x1, y1, fill='white', outline='black')
        

def getHeuristic(node, goal, node_dist_cm):
    dist = math.dist(node.coord, goal.coord) * node_dist_cm
    return dist

def getNeighbors(grid, node, grid_size):
    neighbors = []
    directions = ((0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1))  # Right, Left, Down, Up, Diagonal
    grid_height, grid_width = grid_size

    for dx, dy in directions:
        x, y = node.coord[0] + dx, node.coord[1] + dy
        # Make sure within range
        if 0 <= x < grid_height and 0 <= y < grid_width:
            # Ignore obstacles
            if grid[x, y] == 0:
                new_node = Node(node, (x, y))
                neighbors.append(new_node)

    return neighbors

def astar(grid, start, end):
    grid_size = grid.shape
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
        if current_node.coord == end_node.coord:
            path = []
            while current_node:
                path.append(current_node.coord)
                current_node = current_node.parent
            return path[::-1]  # Return reversed path
        
        neighbors = getNeighbors(grid,current_node,grid_size)
        
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
    

def print_path(map, path):
    grid = map.grid
    exp = map.expanded
    convert_coord = map.grid_to_realworld
    if path:
        for row in range(size(grid,0)):
            for col in range(size(grid,1)):
                if (row, col) in path:
                    print("* ", end="")
                elif grid[row,col] == 1:
                    if (row,col) in exp:
                        print("| ", end="")
                    else:
                        print("X ", end="")
                else:
                    print(". ", end="")
            print()
        print("")
        true_path = [convert_coord(x,y) for x,y in path]
        print(true_path)
    else:
        print("no path found")
        
    print("")
    return true_path

def collinear(p1,p2,p3):
    return (p2[1]-p1[1])*(p3[0]-p2[0]) == (p3[1]-p2[1])*(p2[0]-p1[0])

def simplify_path(path):
    simplified_path = list()
    simplified_path.append(path[0])
    
    for i in range(1,len(path)-1):
        if not collinear(path[i-1],path[i],path[i+1]):
            simplified_path.append(path[i])
    simplified_path.append(path[-1])
    return simplified_path


def main():
    st = time.time()
    map = Map(180,120,2)
    Grid = map.grid
    # root = tk.Tk()
    # root.title("A* Grid Map")

    

    #add boundaries

    map.createObsRect(0,0,3,map.width_cm) #top
    map.createObsRect(0,0,map.height_cm,3) #left
    map.createObsRect(0,(map.width_cm)-3,map.height_cm,(map.width_cm)) #right
    map.createObsRect(map.height_cm,0,map.height_cm-3,map.width_cm) #bottom
    map.createObs(100,75,30,30)
    map.expandObs(12)
    map.addObstacles()

    

    start = (round(16/map.node_dist_cm),round(16/map.node_dist_cm))
    end = ((140//map.node_dist_cm),(60//map.node_dist_cm))

    path = astar(Grid, start, end)
    end = time.time()
    # gui = AStarGridMapGUI(root, map, path)
    # gui.draw_grid_map()
    # root.mainloop()
    
    p = print_path(map,path)
    t = simplify_path(p)
    print(t)
    print("The time of execution of above program is :",(end-st) * 10**3, "ms")
    
   # print(map.grid)

if __name__ == '__main__':
    main()