# We'll Implement Breadth-First Search
import numpy as np
from queue import Queue
from enum import Enum

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid

class Action(Enum):
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)

    def __str__(self):
        if self == self.UP:
            return "^"
        if self == self.DOWN:
            return "v"
        if self == self.LEFT:
            return "<"
        if self == self.RIGHT:
            return ">"

    @property
    def delta(self):
        return (self.value[0], self.value[1])

    @property
    def cost(self):
        return self.value[2]
    
def valid_actions(current_node, grid):
    valid_array = []
    n, m = grid.shape[0] - 1, grid.shape[1] - 1

    for action in Action:
        row = current_node[0] + action.delta[0]
        column = current_node[1] + action.delta[1]

        if (n >= row >= 0) and (m >= column >= 0):
            if grid[row][column] == 0:
                valid_array.append(action)

    return valid_array

def bf_search(grid, start, goal):
    queue = Queue()
    visited = set()
    branch = {}
    
    queue.put(start)
    visited.add(start)
    
    while not queue.empty():
        current_node = queue.get()
        
        for action in valid_actions(current_node, grid):
            row = current_node[0] + action.delta[0]
            column = current_node[1] + action.delta[1]
            new_node = (row, column)
            
            if new_node not in visited:
                queue.put(new_node)
                visited.add(new_node)
                branch[new_node] = (current_node, action)
                
    if goal in branch:
        print("Goal reached!!!")

        path = []
        node = goal

        while branch[node][0] != start:

            path_step = branch[node][0]
            path.append(path_step)

            node = branch[node][0]

        path.append(branch[node][0])

        path = path[::-1]
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
        return
    
    return path

# Prune path
def collinearity(p1, p2, p3, epsilon=1e-6):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    
    area = x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2)
    
    return abs(area) < epsilon

def prune_path(path):
    pruned_path = [p for p in path]

    i = 0
    while i < len(pruned_path)-2:
        p1 = pruned_path[i]
        p2 = pruned_path[i+1]
        p3 = pruned_path[i+2]

        if collinearity(p1, p2, p3):
            pruned_path.pop(i+1)
        else:
            i += 1

    return pruned_path

def create_grid_25(data, safety_distance=0):
    
    north_max = np.ceil(np.amax(data[:,0] + data[:,3]))
    north_min = np.floor(np.amin(data[:,0] - data[:,3]))
    
    east_max = np.ceil(np.amax(data[:,1] + data[:,4]))
    east_min = np.floor(np.amin(data[:,1] - data[:,4]))
    
    alt_max = np.floor(np.amax(data[:,2] - data[:,5]))
    
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))
    
    grid = np.zeros((north_size, east_size))
    
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        
        obstacle = [
            int(north - d_north - north_min - safety_distance),
            int(north + d_north - north_min + safety_distance),
            int(east - d_east - east_min - safety_distance),
            int(east + d_east - east_min + safety_distance),
        ]
        
        height = alt + d_alt + safety_distance
        
        grid[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3]] = height
        
    return grid, north_min, east_min

