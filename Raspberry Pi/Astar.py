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
    def __init__(self,h,w,node_dist):
        self.width_cm = w
        self.height_cm = h
        self.node_dist_cm = node_dist  # Distance between nodes in centimeters

        # Convert dimensions to number of nodes
        self.tile_width = self.width_cm // self.node_dist_cm
        self.tile_height = self.height_cm // self.node_dist_cm

        # Create an empty grid
        self.grid = np.zeros((self.tile_height, self.tile_width))
        self.obstacles = []  # Set of obstacle nodes
        self.expanded = []
    def getGrid(self):
        return self.grid
    def addObstacles(self):
        for row in range(len(self.obstacles)):
            x,y=self.obstacles[row][0],self.obstacles[row][1]
            self.grid[x,y] = 1
        for row in range(len(self.expanded)):
            x,y=self.expanded[row][0],self.expanded[row][1]
            self.grid[x,y] = 1
        
        
    def createObs(self, x, y, w, h):
        x,y = math.floor(x//self.node_dist_cm),math.floor(y//self.node_dist_cm)
        w, h = math.floor(w//self.node_dist_cm),math.floor(h//self.node_dist_cm)
        # newx,newy = int(x-math.ceil(h*0.5)), int(y-math.ceil(w*0.5))
        
        for i in range(-math.ceil(h/2), math.ceil(h/2)):
            for j in range(-math.ceil(w/2), math.ceil(w/2)):
                newx, newy = x + i, y + j
                if 0 <= newx < np.size(self.grid, 0) and 0 <= newy < np.size(self.grid, 1):
                    if (newx,newy) not in self.obstacles:
                        self.obstacles.append((newx, newy))

    def createObsRect(self,x1,y1,x2,y2):
        x1,x2,y1,y2 = x1//self.node_dist_cm,x2//self.node_dist_cm,y1//self.node_dist_cm,y2//self.node_dist_cm
        min_x,max_x = min(x1,x2), max(x1,x2)
        min_y,max_y = min(y1,y2), max(y1,y2)
        for x in range(min_x,max_x):
            for y in range(min_y, max_y):
                if (x,y) not in self.obstacles:
                    self.obstacles.append((x,y))
        
            
    def expandObs(self,dist):
        newobs = []
        directions=[(0,-1),(0,1),(-1,0),(1,0),(-1,-1),(-1,1),(1,-1),(1,1)]
        dist = dist//self.node_dist_cm
        
        for x, y in self.obstacles:
            for i in range(-dist, dist + 1):
                for j in range(-dist, dist + 1):
                    newx, newy = x + i, y + j
                    if 0 <= newx < np.size(self.grid, 0) and 0 <= newy < np.size(self.grid, 1):
                        if (newx, newy) not in newobs and (newx, newy) not in self.obstacles:
                            newobs.append((newx, newy))
                            
        
        self.expanded.extend(newobs)
        
                    
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
        if x > (np.size(grid,0) - 1) or x < 0 or y > (np.size(grid[np.size(grid,0)-1]) -1) or y < 0:
            continue

        # Make sure walkable terrain
        if grid[x,y] != 0:
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
        if current_node.coord == end_node.coord:
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
    

def print_path(map, path):
    
    if path:
        for row in range(np.size(map.grid,0)):
            for col in range(np.size(map.grid,1)):
                if (row, col) in path:
                    print("* ", end="")
                elif map.grid[row,col] == 1:
                    if (row,col) in map.expanded:
                        print("| ", end="")
                    else:
                        print("X ", end="")
                else:
                    print(". ", end="")
            print()
        print("")
        for x, y in path:
            print(map.grid_to_realworld(x,y))
    else:
        print("no path found")
        
    print("")
    
    
def main():
    map = Map(200,150,2)
    Grid = map.grid
    #add boundaries
    map.createObsRect(0,0,3,map.width_cm) #top
    map.createObsRect(0,0,map.height_cm,3) #left
    map.createObsRect(0,map.width_cm-3,map.height_cm,map.width_cm) #right
    map.createObsRect(map.height_cm-3,0,map.height_cm,map.width_cm) #bottom
    map.createObs(100,45,5,5)
    map.expandObs(12)
    map.addObstacles()
    # grid = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
    #         [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #         [0, 0, 0, 0, 1, 1, 0, 0, 0, 0]]

    start = (math.floor(16//map.node_dist_cm-1),math.floor(16//map.node_dist_cm-1))
    end = (math.floor(150//map.node_dist_cm-1),math.floor(50//map.node_dist_cm-1))

    path = astar(Grid, start, end)
    print_path(map,path)
    
    
   # print(map.grid)
    


if __name__ == '__main__':
    main()