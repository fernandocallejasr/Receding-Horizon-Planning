import numpy as np
from numpy import linalg as LA
from planning_utils import create_grid_25
from queue import PriorityQueue
from enum import Enum

def local_voxmap(data, node, side, height, voxel_size=1):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data.
    
    The `voxel_size` argument sets the resolution of the voxel map. 
    """
    north_min = np.floor(node[0]-side/2)
    north_max = np.ceil(node[0]+side/2)
    
    east_min = np.floor(node[1]-side/2)
    east_max = np.ceil(node[1]+side/2)
    
    alt_max = height
    
    north_size = int(np.ceil(north_max - north_min)) // voxel_size
    east_size = int(np.ceil(east_max - east_min)) // voxel_size
    alt_size = int(alt_max) // voxel_size
    
    grid_25, north_offset, east_offset = create_grid_25(data, 3)
    
    # Create an empty grid
    voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)
    
    # minimum and maximum local coordinates in grid 2.5 D
    bottom = int(np.clip(north_min, 0, grid_25.shape[0]-1)) // voxel_size
    top = int(np.clip(north_max, 0, grid_25.shape[0]-1)) // voxel_size
    left = int(np.clip(east_min, 0, grid_25.shape[1]-1)) // voxel_size
    right = int(np.clip(east_max, 0, grid_25.shape[1]-1)) // voxel_size
    alt_max = int(height // voxel_size)
    
    mapping = [bottom, top, left, right]
    
    for i in range(bottom, top):
        for j in range(left, right):
            x = int(np.ceil(i-node[0]+side/2))
            y = int(np.ceil(j-node[1]+side/2))
            
            if grid_25[i][j] != 0:
                height = np.clip(int(grid_25[i][j]), 0, alt_max)
                for h in range(height):
                    voxmap[x][y][h] = True

    return voxmap, mapping

def heuristic3D(position, goal_position):
    position = np.array(position)
    goal_position = np.array(goal_position)
    return LA.norm(position - goal_position)

def getGlobalVoxels(mappingArray, grid_25, alt_max):
    bottom, top, left, right = mappingArray
    height = alt_max
    globalVoxels = []

    for i in range(bottom, top):
        for j in range(left, right):

            if grid_25[i][j] == 0:
                for h in range(height):
                    globalVoxels.append((i,j,h))

    return globalVoxels

def global_to_local_voxel(local_current_node, global_node, side):

    x, y, z = global_node
    
    #x = int(np.ceil(x - local_current_node[0] + side/2))
    #y = int(np.ceil(y - local_current_node[1] + side/2))

    x = int(np.ceil(x - local_current_node[0] + side/2))
    y = int(np.ceil(y - local_current_node[1] + side/2))
    
    return (x, y, z)

def aStar3D(voxmap, start, goal):
    queue = PriorityQueue()
    branch = {}
    visited = set()

    found = False

    visited.add(start)
    queue.put((0, start))

    while queue.empty() == False:
        dequeued = queue.get()
        current_node = dequeued[1]

        if current_node == start:
            cost = 0
        else:
            cost = branch[current_node][0]

        if current_node == goal:
            print("Path Found")
            found = True
            break

        for action in valid_3D_Actions(current_node, voxmap):
            north = current_node[0] + action.delta[0]
            east = current_node[1] + action.delta[1]
            height = current_node[2] + action.delta[2]
            new_node = (north, east, height)
            
            branch_cost = cost + action.cost
            queue_cost = branch_cost + heuristic3D(current_node, goal)

            if new_node not in visited:
                visited.add(new_node)
                queue.put((queue_cost, new_node))
                branch[new_node] = (branch_cost, current_node, action)

    path = []
    path_cost = 0

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][2])
            n = branch[n][1]
        path.append(branch[n][2])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    path = list(path)
        
    return path[::-1], path_cost

class Actions3D(Enum):
    UP = (0, 0, 1, 1)
    DOWN = (0, 0, -1, 1)
    EAST = (0, 1, 0, 1)
    WEST = (0, -1, 0, 1)
    NORTH = (-1, 0, 0, 1)
    SOUTH = (1, 0, 0, 1)

    @property
    def delta(self):
        return (self.value[0], self.value[1], self.value[2])

    @property
    def cost(self):
        return (self.value[3])

def valid_3D_Actions(current_node, voxmap):
    valid_actions = []
    n, m, t = voxmap.shape
    x, y, z = current_node

    for action in Actions3D:
        north = x + action.delta[0]
        east = y + action.delta[1]
        height = z + action.delta[2]

        if (n > north >= 0) and (m > east >= 0) and (t > height >= 0):
            if voxmap[north][east][height] == False:
                valid_actions.append(action)

    return valid_actions

def heuristic3D(position, goal_position):
    position = np.array(position)
    goal_position = np.array(goal_position)
    return LA.norm(position - goal_position)

def global_to_local_voxel(local_current_node, global_node, side):

    x, y, z = global_node
    
    x = int(np.ceil(x - local_current_node[0] + side/2))
    y = int(np.ceil(y - local_current_node[1] + side/2))
    
    return (x, y, z)
