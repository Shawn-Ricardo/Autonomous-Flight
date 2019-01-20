"""

This approach will implement probabilisitc roadmap to obtain an obstacle free
path to a goal within a simulation of San Francisco.

"""

import argparse
import time
import msgpack
from enum import Enum, auto
import sys

import numpy as np

from planning_utils import *

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from skimage.morphology import medial_axis
from skimage.util import invert

from sklearn.neighbors import KDTree

import networkx as nx
from shapely.geometry import Polygon, Point, LineString
from queue import PriorityQueue


class States(Enum):
    """
    The possible states that the drone can exist within.
    """
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):
    """
    Inherit from Drone class. This class interfaces with the flight controller.
    """

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # hold navigation waypoints. these are waypoints obtained from medial-axis graph planning.
        # these will guide the drone to "mini-goals", allowing the drone to perform local planning
        # in order to avoid sudden obstacles, environment hazards, and/or any differentiations
        # in the map/environment.
        self.navigation_waypoints = []

        # a list of polygons. create a KDTree that uses the centroid of the
        # polygon to sort in space, and also appends the polygon itself and the
        # height of the polygon.
        self.polygons = 0
        self.polygon_centroids = 0
        self.polygon_centroids_tree = 0

        self.grid = 0

        # gate for finite state machine
        self.initial_run = True

        # lets us know when the final navigation point is within the local planner so that we can
        # stop planning.
        # ideally, this value should become true when at the final location.
        self.stop_planning = False

        # target altitude
        self.TARGET_ALTITUDE = 0

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)



    def local_position_callback(self):

        if self.flight_state == States.TAKEOFF:

            # if we have reached target altitude, call waypoint_transitition
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:

            # check if the drone is within 1 meter of the target position.
            # this is the deadband.
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 10.0:

                # if there are more waypoints to fly to, call a function that will publish the
                # waypoint.
                if (len(self.waypoints) > 0 or len(self.navigation_waypoints) > 0):
                    self.waypoint_transition()

                else:

                    # no waypoints to go toward, check if I have hovering in order to land
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()


    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()



    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):

        """

        Call the local and global planner to obtain
              the next waypoint.

        """

        self.plan_path()

        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):

        # gateway. If a local path is found, will publish waypoints.
        # if a local path is not found, will attempt to find a path
        # again.
        path_found = False

        while not self.stop_planning and not path_found:

            # if this is the very first run, plan a path using medial-axis graph.
            # this planning occurs very quickly and will serve as a guiding set of waypoints for the drone's
            # local planner.
            if self.initial_run:

                navigation_path_found = False

                # continue to iterate until we have found a path using medial-axis.
                while not navigation_path_found:

                    self.flight_state = States.PLANNING

                    print("Searching for navigation path ...")

                    # set target altitude and safety distance to obstacles.
                    self.TARGET_ALTITUDE = 1

                    # a safety distance of 8 works very well given the limitations of the simulation.
                    # Do not alter this value.
                    SAFETY_DISTANCE = 8
                    self.target_position[2] = self.TARGET_ALTITUDE

                    # read lat0, lon0 from colliders into floating point values,
                    # and set home position to these values.
                    # the home position is not the current position of the drone, but a predefined
                    # location.
                    with open('colliders.csv') as f:
                        first_line = f.readline()
                        item_1, item_2 = first_line.split(', ')
                        lat = float(item_1.split(' ')[1])
                        lon = float(item_2.split(' ')[1])
                        self.set_home_position(lon,lat,0.0)
                        print('extracted home lat and lon from colliders')

                    # retrieve current global position (in latitude and longitude)
                    curr_global_pos = self.global_position

                    # convert global position to local position.
                    curr_local_position = global_to_local(curr_global_pos, self.global_home)

                    # print out this useful information for debug
                    #print('global home {0}, global position {1}, local position {2}'.format(self.global_home, self.global_position, self.local_position))

                    # Read in 2.5D obstacle map.
                    # NOTE: this obstacle map IS NOT 1:1 with simulation. That is, the
                    # obstacles in this file are not laid out exactly like the simulator.
                    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

                    # Using the obstacle data, create a GRID using a target altitude and safety margin around obstacles.
                    grid, north_offset, east_offset = create_grid(data, self.TARGET_ALTITUDE, SAFETY_DISTANCE)
                    self.north_offset = north_offset
                    self.east_offset = east_offset
                    #print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

                    # run medial axis on the inverted grid to find a graph.
                    # for more information on medial axis, view the readme.
                    skeleton = medial_axis(invert(grid))

                    # set grid start position to current position
                    grid_start = (-north_offset + int(curr_local_position[0]), -east_offset + int(curr_local_position[1]))

                    # check if LAT/LON arguments are passed in for a goal.
                    grid_goal = 0
                    lat, lon = args.lat, args.lon

                    if lat == '' or lon == '':
                        # no latitude and longitude goals passed in.
                        # here are some hardcoded goals.
                        # have only 1 goal activate at a time.
                        print('No latitude or longitude value passed in. Will fly to hardcoded goal location')
                        grid_goal = (815, 221)
                        #grid_goal = (650, 60)
                        #grid_goal = (612, 540)
                        #grid_goal = (381, 331)
                        #grid_goal = (86, 846)
                        #grid_goal = (337, 812)

                    else:

                        print('goal latitude and longitude read from command line')

                        # convert string to float
                        lat, lon = np.float64(lat), np.float64(lon)

                        # long, lat, altitude
                        goal_global_position = np.array([lon, lat, 0.082])

                        # convert global coordinates to local coordinates
                        goal_local_position = global_to_local(goal_global_position, self.global_home)

                        # find goal in grid using offsets obtains above.
                        grid_goal = (-north_offset + int(goal_local_position[0]), -east_offset + int(goal_local_position[1]))

                    # find nearest points to graph skeleton from grid start/stop locations
                    skel_start, skel_goal = find_start_goal(skeleton, grid_start, grid_goal)

                    # Run A* to find a path from start to goal
                    print('Local Start and Goal: ', skel_start, skel_goal)
                    # the heuristic is a simple euclidean distance.
                    navigation_path_found, path, _ = a_star_medial_axis(invert(skeleton).astype(np.int), heuristic_euc, tuple(skel_start), tuple(skel_goal))

                    if navigation_path_found:

                        # prune path to minimize number of waypoints.
                        # run twice. will improve resulting waypoint sequence.
                        print('pruning global path')
                        path = path_prune(path, grid)
                        path = path_prune(path, grid)

                        # Convert global path to waypoints
                        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), int(self.TARGET_ALTITUDE), 0] for p in path]

                        # Set self.navigation_waypoints. These waypoints will be used as a GUIDE for receding horizon planning.
                        # Start from current location.
                        self.navigation_waypoints = waypoints[1:]
                        self.waypoints = waypoints

                        # extract polyons and create a KDTree from the returned
                        # list of polygon centroids.
                        self.polygons, self.polygon_centroids = extract_polygons(data, SAFETY_DISTANCE)
                        self.polygon_centroids_tree = KDTree(self.polygon_centroids)

                        # set up work has completed.
                        self.initial_run = False

                        # send guiding waypoints to simulator for visualization
                        self.send_waypoints()

                        # waypoints will be populated by the local planner. Not this sequence, which obtains the
                        # global plan
                        self.waypoints = []

                    else:

                        print('could not find navigational path')


            # at this point, the global plan exists.
            # will implent receding horizon local planner using Probabilistic Roadmap.

            temp_local_position = 0

            # obtain the local position
            global_position = self.global_position
            local_position = global_to_local(global_position, self.global_home)

            if sum(self.target_position) == 1:
                # if there is no target position set, make the current location of the drone the target position.
                temp_local_position = [int(local_position[0]), int(local_position[1])]
            else:
                # plan to the current target position.
                temp_local_position = [int(self.target_position[0]), int(self.target_position[1])]

            # the first navigation waypoint in the list will serve as a guide to the local planner.
            # extract just the (x,y).
            temp_goal = self.navigation_waypoints[0]
            temp_goal = [temp_goal[0], temp_goal[1]]

            # find distance between the target waypoint and the navigation waypoint.
            dist_nav_goal = heuristic_euc(temp_local_position, temp_goal)

            # if the drone is close to the navigation waypoint and there are greater than 1 navigation waypoints in the list,
            # pop the first navigation waypoint and use the next waypoint as a guide.
            if dist_nav_goal < 10.0 and len(self.navigation_waypoints) > 1:
                self.navigation_waypoints.pop(0)
                print('POPPING WAYPOINT')


            # get k samples within local space.
            # the horizon is defined within the get_samples() function.
            num_samples = 500
            samples = get_samples(temp_local_position, num_samples)

            # using KD tree containing obstacles, remove all nodes that exist within, or close to, an obstacle.
            # will return a list that contains nodes in the free space.
            nodes = remove_collisions(samples, self.polygons, self.polygon_centroids, self.polygon_centroids_tree, self.TARGET_ALTITUDE)

            """
            Uncomment this section to visualize the nodes that exist in free space.

            new_list = []

            for p in nodes:
                new_node = [int(p[0]), int(p[1]), int(self.TARGET_ALTITUDE), 0]
                new_list.append(new_node)

            hold_waypoints = self.waypoints
            self.waypoints = new_list
            self.send_waypoints()
            self.waypoints = hold_waypoints
            """


            # create a graph from these nodes.
            connections_per_node = 5
            g, _ = create_graph(nodes, self.polygons, self.polygon_centroids, self.polygon_centroids_tree, connections_per_node, self.TARGET_ALTITUDE)

            # find closest graph nodes to start position and goal position
            nav_point = [int(self.navigation_waypoints[0][0]), int(self.navigation_waypoints[0][1])]
            start_point = [int(temp_local_position[0]), int(temp_local_position[1])]
            dist_node_to_goal = 1000000
            dist_node_to_start = 1000000
            goal_node = 0
            start_node = 0

            for node in list(g.nodes):

                iteration_goal_dist = heuristic_euc(node, nav_point)
                iteration_start_dist = heuristic_euc(node, start_point)

                if iteration_goal_dist < dist_node_to_goal:
                    dist_node_to_goal = iteration_goal_dist
                    goal_node = node

                if iteration_start_dist < dist_node_to_start:
                    dist_node_to_start = iteration_start_dist
                    start_node = node

            """
            Uncomment this section to visualize the nodes that exist in free space.

            new_list = []

            for p in nodes:
                new_node = [int(p[0]), int(p[1]), int(self.TARGET_ALTITUDE), 0]
                new_list.append(new_node)

            hold_waypoints = self.waypoints
            self.waypoints = new_list
            self.send_waypoints()
            self.waypoints = hold_waypoints
            """

            # perform a* on the graph using the goal node and start node
            path_found, path, _ = a_star_graph(g, heuristic_euc, start_node, goal_node)

            if path_found:

                # minimize the number of waypoints found. Produces a smoother path.
                print('pruning local graph')
                path = graph_prune(path, self.polygons, self.polygon_centroids, self.polygon_centroids_tree, self.TARGET_ALTITUDE)

                # set up waypoints for publishing to simulator
                waypoints = [[int(p[0]), int(p[1]), int(self.TARGET_ALTITUDE), 0] for p in path]
                self.waypoints = waypoints[1:]
                self.send_waypoints()

                # check if we are at the final navigation point (our destination)
                temp_local_position = [int(self.local_position[0]), int(self.local_position[1])]
                temp_goal = self.navigation_waypoints[0]
                temp_goal = [temp_goal[0], temp_goal[1]]
                dist_nav_goal = heuristic_euc(temp_local_position, temp_goal)

                if dist_nav_goal < 5 and len(self.navigation_waypoints) == 1:
                    # distance to goal is less than local planner and it is the last waypoint. stop planning here.
                    # pop last waypoint.
                    # If navigation_waypoints[] or waypoints[] are empty, landing_transition is signaled.
                    self.navigation_waypoints.pop(0)
                    self.stop_planning = True
                    print('stopping planning')

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int,   default=5760,        help='Port number')
    parser.add_argument('--host', type=str,   default='127.0.0.1', help="host address, i.e. '127.0.0.1'")

    # Add in command line option to pass in latitude and longitude
    parser.add_argument('--lat',  type=str, default='',         help='')
    parser.add_argument('--lon',  type=str, default='',         help='')
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
