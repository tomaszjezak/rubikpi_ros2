"""
Probabilistic Roadmap (PRM) Core Algorithm
Inspired heavily from PythonRobotics PRM implementation
"""

""" 
think about

- dynamically changing how far from wall we want to be (ie safe vs time/dist min)
"""


import math
import numpy as np
from scipy.spatial import KDTree

# Parameters

N_SAMPLE = 1000 # number of sample points to generate on the 10x10 map
MAX_EDGE_LEN = 14.1 # hypotenuse of grid ie. 14.1 in a 10x10

class Node:
    """
    Node class for dijkstra search
    """
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + \
               str(self.cost) + "," + str(self.parent_index)


def sample_points(sx, sy, gx, gy, rr, workspace_bounds, obstacle_bounds, obstacle_kd_tree, rng):
    """
    Sample random points in free space... 25% extra credit baby
    Ensure points are collision-free in the obstacle-bounded space.
    These points are used downstream to build the roadmap
    
    Args:
        sx: Start x coordinate [m]
        sy: Start y coordinate [m]
        gx: Goal x coordinate [m]
        gy: Goal y coordinate [m]
        rr: Robot radius [m] - used to check collision-free space
        workspace_bounds: Tuple (min_x, max_x, min_y, max_y) defining workspace
        obstacle_bounds: Tuple (min_x, max_x, min_y, max_y) defining obstacle region
        obstacle_kd_tree: KDTree object for efficient nearest neighbor queries
        rng: Optional random number generator for reproducibility
    
    Returns:
        sample_x: List of sampled x coordinates (includes start and goal)
        sample_y: List of sampled y coordinates (includes start and goal)   
    """
    # unpack workspace bounds
    min_x, max_x, min_y, max_y = workspace_bounds
    
    # Unpack obstacle bounds
    obs_min_x, obs_max_x, obs_min_y, obs_max_y = obstacle_bounds
        
    sample_x, sample_y = [], []

    if rng is None:
        rng = np.random.default_rng()
    
    while len(sample_x) <= N_SAMPLE:
        tx = (rng.random() * (max_x - min_x)) + min_x
        ty = (rng.random() * (max-y - min_y)) + min_y

        # geomtric check, skip sample if inside obstacle
        if obs_min_x <= tx <= obs_max_x and obs_min_y <= ty <= obs_max_y:
            continue
        
        # KDTree check: ensure that sample is at least 'rr' distance from Middle Obstacle AND Walls
        dist, index = obstacle_kd_tree.query([tx,ty])

        if dist >= rr: # point is safe, at least 'rr' distance from nearest obstacles
            sample_x.append(tx)
            sample_y.append(ty)

    # append start and goal points
    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y

def is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree):
    """
    Check if an edge between two points collides with obstacles.
    
    This function checks if a straight-line edge from (sx, sy) to (gx, gy) 
    would collide with any obstacles, considering the robot's radius. It does
    this by discretely sampling points along the edge and checking if each
    point is too close to obstacles.

    Returns:
        bool: True if collison detected, False if edge is collision-free
    """
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.hypot(dx, dy)

    if d >= MAX_EDGE_LEN:
        return True
    
    D = rr # D : step size is robot radius
    n_step = round(d / D) # number of steps along the edge

    # check points along the edge, is there collision
    for i in range(n_step):
        dist, _ = obstacle_kd_tree.query([x,y])
        if dist <= rr:
            return True # collision with wall/obstacle
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # goal point check
    dist, _ = obstacle_kd_tree.query([gx, gy])
    if dist <= rr:
        return True # collision
    
    return False # no collision

    """ Tomasz: make sure this func makes sense tomorrow babe"""