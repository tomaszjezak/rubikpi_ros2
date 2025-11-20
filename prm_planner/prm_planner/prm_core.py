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
N_KNN = 10 # number of edges from one sampled point, for K-Nearest Neighbors to connect

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
        ty = (rng.random() * (max_y - min_y)) + min_y

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
        x += D * math.cos(yaw) # move forward a step (compute if its collision in next iteration of for loop)
        y += D * math.sin(yaw)

    # goal point check
    dist, _ = obstacle_kd_tree.query([gx, gy])
    if dist <= rr:
        return True # collision
    
    return False # no collision

def generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree):
    """
    Generate a roadmap by connecting sample points with collision-free edges.
    
    For each sample point, finds the N_KNN nearest neighbors and connects
    them if the edge is collision-free.
    
    Args:
        sample_x: List of x coordinates of sampled points
        sample_y: List of y coordinates of sampled points  
        rr: Robot radius [m]
        obstacle_kd_tree: KDTree for obstacle collision checking
    
    Returns:
        road_map: List of lists, where road_map[i] contains indices of 
                 connected neighbors for sample point i
    """

    road_map = [] # each element will be a list of neighbor indices for that sample point
    n_sample = len(sample_x)
    sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T) # kdtree enables fast nearest-neighbors queries

    for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

        dists, indices = sample_kd_tree.query([ix, iy], k=n_sample) # get node i's nearest neighbors (sorted list, closest first)
        edge_id = []

        for ii in range(1, len(indices)): # loop through neighbors, start at index 1 (skip point itself)
            nx = sample_x[indices[ii]] 
            ny = sample_y[indices[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obstacle_kd_tree):
                edge_id.append(indices[ii])

            if len(edge_id) >= N_KNN: # stop onc we've found N_KNN collision-free neighbors
                break

        road_map.append(edge_id) # add this point's list of connected neighbors to roadmap
                                 # road_map[i] := list of neighbor indices that point i can connect to
    return road_map 


def dijkstra_planning(sx, sy, gx, gy, road_map, sample_x, sample_y):
    """
    Find shortest path from start to goal using Dijkstra's algorithm on the roadmap.
    
    Uses Dijkstra's algorithm to search through the roadmap graph, finding the
    minimum-cost path from the start position to the goal position. The algorithm
    explores nodes in order of increasing cost, ensuring the first path found to
    the goal is optimal.
    
    Assumes start and goal are the last two points in sample_x/sample_y:
    - Start is at index len(sample_x) - 2
    - Goal is at index len(sample_x) - 1
    
    Args:
        sx: Start x coordinate [m]
        sy: Start y coordinate [m]
        gx: Goal x coordinate [m]
        gy: Goal y coordinate [m]
        road_map: List of lists, where road_map[i] contains indices of 
                 connected neighbors for sample point i
        sample_x: List of x coordinates of all sampled points [m]
        sample_y: List of y coordinates of all sampled points [m]
    
    Returns:
        Tuple of (rx, ry) where:
        - rx: List of x coordinates of path waypoints (goal to start)
        - ry: List of y coordinates of path waypoints (goal to start)
        - Returns ([], []) if no path is found
    """

    start_node = Node(sx, sy, 0.0, -1)
    goal_node = Node(gx, gy, 0.0, -1)

    open_set, closed_set = dict(), dict() # key: node IDs (indices) ; values: Node objects
    open_set[len(road_map) - 2] = start_node

    path_found = True

    while True:
        if not open_set:
            print("Cannot find path")
            path_found = False
            break
        c_id = min(open_set, key=lambda o: open_set[o].cost) # find the node ID in open_set with the lowest cost
        current = open_set[c_id] # expand the lowest cost node next

        # Check if goal isreached
        if c_id == (len(road_map) - 1):
            print("Goal is found")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        # remove item from open set, add it to closed set
        del open_set[c_id]
        closed_set[c_id] = current

        # expand search grid based on motion model
        for i in range(len(road_map[c_id])): #explore neighbors of current node, expanding search
            n_id = road_map[c_id][i] # neighbor id - get neighbor index at position i
            dx = sample_x[n_id] - current.x # compute deltas from current node to neighbor
            dy = sample_y[n_id] - current.y
            d = math.hypot(dx, dy)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

            if n_id in closed_set:
                continue
            if n_id in open_set:
                if open_set[n_id].cost > node.cost:
                    open_set[n_id].cost = node.cost
                    open_set[n_id].parent_index = c_id
            else:
                open_set[n_id] = node

    if path_found is False:
        return [], []

    # generate final course by backtracking from goal to start
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry