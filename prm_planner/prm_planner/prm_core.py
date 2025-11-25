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
from scipy.spatial import KDTree  # Still needed for sample_kd_tree in generate_road_map


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


def distance_to_box(x, y, obstacle_bounds):
    """
    Calculate distance from point (x, y) to nearest edge of box obstacle.
    
    Args:
        x: X coordinate of point
        y: Y coordinate of point
        obstacle_bounds: Tuple (min_x, max_x, min_y, max_y) defining box
    
    Returns:
        float: Distance to nearest box edge (0 or negative if inside box)
    """
    obs_min_x, obs_max_x, obs_min_y, obs_max_y = obstacle_bounds
    
    # Check if point is inside box
    if obs_min_x <= x <= obs_max_x and obs_min_y <= y <= obs_max_y:
        # Inside box - return 0 (will be rejected by geometric check anyway)
        return 0.0
    
    # Outside box - distance to nearest edge
    # Calculate horizontal distance to box
    if x < obs_min_x:
        dx = obs_min_x - x  # Left of box
    elif x > obs_max_x:
        dx = x - obs_max_x  # Right of box
    else:
        dx = 0  # Between left and right edges
    
    # Calculate vertical distance to box
    if y < obs_min_y:
        dy = obs_min_y - y  # Below box
    elif y > obs_max_y:
        dy = y - obs_max_y  # Above box
    else:
        dy = 0  # Between top and bottom edges
    
    # Distance to nearest edge/corner
    return math.sqrt(dx*dx + dy*dy)


def sample_points(sx, sy, gx, gy, rr, workspace_bounds, obstacle_bounds, rng, n_sample=400):
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
        rng: Optional random number generator for reproducibility
        n_sample: Number of sample points to generate (default: 1000)

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

    while len(sample_x) <= n_sample:
        tx = (rng.random() * (max_x - min_x)) + min_x
        ty = (rng.random() * (max_y - min_y)) + min_y

        # geometric check, skip sample if inside obstacle
        if obs_min_x <= tx <= obs_max_x and obs_min_y <= ty <= obs_max_y:
            continue
        
        # Distance check: ensure that sample is at least 'rr' distance from box obstacle
        dist = distance_to_box(tx, ty, obstacle_bounds)

        if dist >= rr:  # point is safe, at least 'rr' distance from nearest obstacles
            sample_x.append(tx)
            sample_y.append(ty)

    # append start and goal points
    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y

def is_collision(sx, sy, gx, gy, rr, obstacle_bounds, max_edge_len=14.1):
    """
    Check if an edge between two points collides with obstacles.

    This function checks if a straight-line edge from (sx, sy) to (gx, gy)
    would collide with any obstacles, considering the robot's radius. It does
    this by discretely sampling points along the edge and checking if each
    point is too close to obstacles.

    Args:
        sx: Start x coordinate [m]
        sy: Start y coordinate [m]
        gx: Goal x coordinate [m]
        gy: Goal y coordinate [m]
        rr: Robot radius [m]
        obstacle_bounds: Tuple (min_x, max_x, min_y, max_y) defining box obstacle
        max_edge_len: Maximum edge length allowed (default: 14.1)

    Returns:
        bool: True if collision detected, False if edge is collision-free
    """
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.hypot(dx, dy)

    if d >= max_edge_len:
        return True
    
    D = rr # D : step size is robot radius
    n_step = round(d / D) # number of steps along the edge

    # check points along the edge, is there collision
    for i in range(n_step):
        dist = distance_to_box(x, y, obstacle_bounds)
        if dist <= rr:
            return True  # collision with wall/obstacle
        x += D * math.cos(yaw)  # move forward a step
        y += D * math.sin(yaw)

    # goal point check
    dist = distance_to_box(gx, gy, obstacle_bounds)
    if dist <= rr:
        return True  # collision
    
    return False  # no collision

def generate_road_map(sample_x, sample_y, rr, obstacle_bounds, n_knn=10, max_edge_len=14.1):
    """
    Generate a roadmap by connecting sample points with collision-free edges.

    For each sample point, finds the n_knn nearest neighbors and connects
    them if the edge is collision-free.

    Args:
        sample_x: List of x coordinates of sampled points
        sample_y: List of y coordinates of sampled points
        rr: Robot radius [m]
        obstacle_bounds: Tuple (min_x, max_x, min_y, max_y) defining box obstacle
        n_knn: Number of nearest neighbors to connect (default: 10)
        max_edge_len: Maximum edge length allowed (default: 14.1)

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

            if not is_collision(ix, iy, nx, ny, rr, obstacle_bounds, max_edge_len):
                edge_id.append(indices[ii])

            if len(edge_id) >= n_knn: # stop once we've found n_knn collision-free neighbors
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