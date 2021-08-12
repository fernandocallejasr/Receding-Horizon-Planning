import matplotlib.pyplot as plt
import numpy as np

def graph_voxmap(voxmap):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.voxels(voxmap, edgecolor='k')
    ax.set_xlim(voxmap.shape[0], 0)
    ax.set_ylim(0, voxmap.shape[1])
    # add a bit to z-axis height for visualization
    ax.set_zlim(0, voxmap.shape[2])

    plt.xlabel('North')
    plt.ylabel('East')

    plt.show()
    
def graph_voxmap_target(voxmap, target):
    
    new_voxmap = voxmap.copy()
    
    tx, ty, tz = target
    new_voxmap[tx][ty][tz] = True
    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.voxels(new_voxmap, edgecolor='k')
    
    ax.set_xlim(new_voxmap.shape[0], 0)
    ax.set_ylim(0, new_voxmap.shape[1])
    # add a bit to z-axis height for visualization
    ax.set_zlim(0, new_voxmap.shape[2])
    
    plt.xlabel('North')
    plt.ylabel('East')

    plt.show()

def graph_pruned_path_node(grid, pruned_path, idx, start_ne, goal_ne):
    plt.rcParams['figure.figsize'] = 6, 6
    
    plt.imshow(grid, cmap='Greys', origin='lower')

    # For the purposes of the visual the east coordinate lay along
    # the x-axis and the north coordinates long the y-axis.
    plt.plot(start_ne[1], start_ne[0], 'x')
    plt.plot(goal_ne[1], goal_ne[0], 'x')
    plt.plot(pruned_path[idx][1], pruned_path[idx][0], 'x')

    pp = np.array(pruned_path)
    plt.plot(pp[:, 1], pp[:, 0], 'g')

    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()

