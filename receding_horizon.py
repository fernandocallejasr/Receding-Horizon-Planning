import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.neighbors import KDTree

from planning_utils import bf_search, create_grid_25, prune_path, create_grid
from planning_3D_utils import aStar3D, global_to_local_voxel, getGlobalVoxels, local_voxmap
from graph_utils import graph_voxmap, graph_voxmap_target

from queue import Queue
from enum import Enum

# Load Data
filename = 'colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)

# Set constraints
flight_altitude = 3
safety_distance = 3
grid = create_grid(data, flight_altitude, safety_distance)

# Find Path
start_ne = (25,  100)
goal_ne = (750., 370.)
path = bf_search(grid, start_ne, goal_ne)

# Prune Path
pruned_path = prune_path(path)
PP_NODE = 16

# Create Voxel
VOXEL_SIDE = 5
VOXEL_SIDE -= 1
VOXEL_HEIGHT = 5

voxmap, mapping = local_voxmap(data, pruned_path[PP_NODE], VOXEL_SIDE, VOXEL_HEIGHT)
grid_25, _, _ = create_grid_25(data, safety_distance)

globalVoxels = getGlobalVoxels(mapping, grid_25, VOXEL_HEIGHT)

# Find local target
# Usamos un KD Tree para encontrar el punto más cercano dentro del voxel local
voxelTree = KDTree(globalVoxels)
nextWaypoint = (pruned_path[PP_NODE+1][0], pruned_path[PP_NODE+1][1], 0)

dist, idxs = voxelTree.query([nextWaypoint], k=1, return_distance=True)

# Obtain Local Partial Target
global_partial_goal = globalVoxels[idxs[0][0]]
local_target = global_to_local_voxel(pruned_path[PP_NODE], global_partial_goal, VOXEL_SIDE)
local_target = np.array(local_target)

# Graph Voxmap and VOxmap with target
graph_voxmap(voxmap)
graph_voxmap_target(voxmap, local_target)

# Obtain map in 3D Voxel
global_current = (pruned_path[PP_NODE][0], pruned_path[PP_NODE][1], 0)

local_current_point = global_to_local_voxel(pruned_path[PP_NODE], global_current, VOXEL_SIDE)

aStar3D(voxmap, local_current_point, tuple(local_target))