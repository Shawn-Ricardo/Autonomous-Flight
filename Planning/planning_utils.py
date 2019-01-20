from enum import Enum
from queue import PriorityQueue
import numpy as np
from shapely.geometry import Polygon, Point, LineString
import networkx as nx
import numpy.linalg as LA
from sklearn.neighbors import KDTree
import random

"""

This file contains many functions to implement A*, medial axis graph, and probabilistic roadmap.

"""


"""

get_samples() will take in a current position and generate k samples within a grid.
then, using rejection sampling, these values will be eliminated if they do not fall
within a radius of the drone.

Sampling in this manner will produce a circle as a local horizon. Making the points
in our horizon react easier to the environment.

"""
def get_samples(position, k):

    # use rejection sampling in order to produce a circular region.
    # generate a square region, then reject all samples that do not fall
    # within a defined circle.

    # create grid.
    xmin = int(position[0]) - 15
    xmax = int(position[0]) + 15
    ymin = int(position[1]) - 15
    ymax = int(position[1]) + 15

    xvals = []
    yvals = []

    for _ in range(k):
        xvals.append(random.randint(xmin, xmax))
        yvals.append(random.randint(ymin, ymax))

    samples = list(zip(xvals,yvals))

    new_list = []

    for sample in samples:

        euc_dist = heuristic_euc(sample, [position[0], position[1]])
        if (euc_dist < 15):
            new_list.append(sample)

    return new_list

"""
return a set of nodes that are within configuration space and obstacle free

"""

def remove_collisions(samples, polygons, centroids, centroids_tree, altitude):

    # a set that will hold nodes in free space
    nodes = set()

    for sample in samples:

        obstructed = False

        # get indices of the three closest centroids to this sample.
        indices = centroids_tree.query([sample], k=10, return_distance=False)[0]

        # the index of the centroid is also the index of the corresponding polygon.
        for index in indices:

            # access the corresponding polygon
            polygon, height = polygons[index]

            if polygon.contains(Point(sample)) and height >= altitude:
                obstructed = True
                break

        if not obstructed:
            nodes.add(sample)

    return list(nodes)


"""

using a safety list of obstacles and a safety distance, construct polygons.

"""
def extract_polygons(data, safety):

    polygons = []
    centroids = []

    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        obstacle = [north - (d_north + safety) , north + (d_north + safety), east - (d_east + safety), east + (d_east + safety)]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]

        height = alt + d_alt

        p = Polygon(corners)
        polygons.append((p, height))
        # append the corresponding centroid (both have the same index)
        centroids.append((p.centroid.x, p.centroid.y))

    return polygons, centroids

"""

same concept as path_prune(). reduce the number of waypoints in the path as much as possible.

"""
def graph_prune(path, polygons, centroids, centroids_tree, altitude):


    if len(path) < 3:
        print('cannot short graph path')
        return path

    new_path = []
    new_path.append(path[0])
    current_node = new_path[-1]
    counter = 1

    while counter < (len(path) - 1):

        current_node = new_path[-1]
        new_node = path[counter]

        if not can_connect(current_node, new_node, polygons, centroids, centroids_tree, altitude):
           new_path.append(path[counter-1])
        else:
            counter += 1

    new_path.append(path[-1])
    return new_path


"""

will test to see if a connection between two nodes intersects an obstacle.

"""
def can_connect(n1, n2, polygons, centroids, centroids_tree, altitude):

    # find nearest polygon indexes to these points
    nearest_n1 = centroids_tree.query([n1], k=5, return_distance=False)[0]
    nearest_n2 = centroids_tree.query([n2], k=5, return_distance=False)[0]

    # consolidate points. there may be duplicates, so use a set.
    polygon_set = set()

    for item in nearest_n1:
        if item not in polygon_set:
            polygon_set.add(item)
    for item in nearest_n2:
        if item not in polygon_set:
            polygon_set.add(item)

    # create a line between nodes
    l = LineString([n1,n2])

    for index in polygon_set:

        # obtain polygon from index
        current_polygon, height = polygons[index]

        # check for any violation.
        if current_polygon.crosses(l) and height >= altitude:
            return False

    return True

"""

create a graph from nodes that lie in free space.
parameter k describes how many connections for a particular node.


"""

def create_graph(nodes, polygons, centroids, centroids_tree, k, altitude):

    # initialize networkx graph
    g = nx.Graph()

    # create a KDTree of the nodes that lie in free space
    tree = KDTree(nodes)

    # for each node, try to connect up to k nearest neighbors
    for n1 in nodes:

        # obtain indices of nodes array that are nearest neighbors to node n1.
        # need to include k + 1 because the tree will return the exact point, as well.
        idxs = tree.query([n1], k + 1, return_distance=False)[0]

        if len(idxs) > 0:

            for idx in idxs:

                n2 = nodes[idx]

                # continue processing if current node is equal to the node
                # we are analyzing
                if np.all(n2 == n1):
                    continue

                # check if a line between these nodes intersects a polygon.
                if can_connect(n1, n2, polygons, centroids, centroids_tree, altitude):
                    g.add_edge((n1[0],n1[1]),(n2[0],n2[1]), weight=1)

    return g, tree


# the classroom function assumes that for all points x1 and x2, it is
# the case that x1 < x2 and y1 < y2.
# This is not the case in the real world. we need to include all combinations of x1 and x2.
# this class was obtained from github, at the following URL address:
# https://gist.github.com/flags/1132363

class bresenham:
    def __init__(self, start, end):
        self.start = list(start)
        self.end = list(end)
        self.path = []

        self.steep = abs(self.end[1]-self.start[1]) > abs(self.end[0]-self.start[0])

        if self.steep:
            self.start = self.swap(self.start[0],self.start[1])
            self.end = self.swap(self.end[0],self.end[1])

        if self.start[0] > self.end[0]:

            _x0 = int(self.start[0])
            _x1 = int(self.end[0])
            self.start[0] = _x1
            self.end[0] = _x0

            _y0 = int(self.start[1])
            _y1 = int(self.end[1])
            self.start[1] = _y1
            self.end[1] = _y0

        dx = self.end[0] - self.start[0]
        dy = abs(self.end[1] - self.start[1])
        error = 0
        derr = dy/float(dx)

        ystep = 0
        y = self.start[1]

        if self.start[1] < self.end[1]: ystep = 1
        else: ystep = -1

        for x in range(self.start[0],self.end[0]+1):
            if self.steep:
                self.path.append((y,x))
            else:
                self.path.append((x,y))

            error += derr

            if error >= 0.5:
                y += ystep
                error -= 1.0

    def swap(self,n1,n2):
        return [n2,n1]

"""
Will consolidate waypoints

Class bresenham will return a list of cells that connect two points.
Analyze this list for the presence of obstacles.

"""
def path_prune(path, grid):

    # check if there are at least 3 points in path
    if (len(path) < 3):
        print('cannot shorten path')
        return path

    new_path = []
    new_path.append(path[0])
    counter = 1

    while counter <= (len(path) - 1):
        current_point = new_path[-1]
        new_point = path[counter]
        cells = bresenham(current_point, new_point)

        for cell in cells.path:

            if grid[cell[0], cell[1]] == 1:
                # obstacle present between these points.
                # add prior point to the new list and start process again from
                # this point.
                if path[counter - 1] not in new_path:
                    new_path.append(path[counter - 1])
                break

        # path between points is clear, analyze a new pair of points.
        counter += 1

    # append goal point and return path
    new_path.append(path[-1])
    return new_path

"""

will find closest corresdponding points on a medial-axis graph

"""
def find_start_goal(skel, start, goal):
    skel_cells = np.transpose(skel.nonzero())
    start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells), axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]

    return near_start, near_goal

"""

Returns a grid representation of a 2D configuration space
based on given obstacle data, drone altitude and safety distance
arguments.

"""
def create_grid(data, drone_altitude, safety_distance):


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

    return grid, int(north_min), int(east_min)

class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    # for diagonals
    NORTH_WEST = (-1, -1, 1.4142)
    NORTH_EAST = (-1, 1, 1.4142)
    SOUTH_WEST = (1, -1, 1.4142)
    SOUTH_EAST = (1, 1, 1.4142)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    # diagonal conditions
    if x - 1 < 0 or y + 1 > m or grid[x-1, y+1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if x - 1 < 0 or y - 1 < 0 or grid[x-1,y-1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if x + 1 > n or y + 1 > m or grid[x+1,y+1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    if x + 1 > n or y - 1 < 0 or grid[x+1,y-1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)

    return valid_actions


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        current_cost = item[0]

        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost

"""

Similar to A* above, just returns a boolean if a path is found or not.

"""
def a_star_medial_axis(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        current_cost = item[0]

        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    return found, path[::-1], path_cost

"""
A* algorithm designed to run on a networkx graph.

"""
def a_star_graph(graph, heuristic, start, goal):

    path = []
    queue = PriorityQueue()
    queue.put((0,start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found path')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node)

    path_cost = 0

    if found:
        n = goal
        path_cost = branch[n][0]

        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]

        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    return found, path[::-1], path_cost



def heuristic_euc(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def heuristic_graph(n1, n2):
    return LA.norm(np.array(n2) - np.array(n1))


