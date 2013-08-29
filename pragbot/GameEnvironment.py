#!/usr/bin/env python

import heapq
import math
from threading import Lock

ROW_DELIMITER = ';'
DEFAULT_MAP = 'pragbot/Maps/ScenarioEnv.txt'
STARTING_AREA = "entrance"

class Node:
    """"Node for A* pathing"""
    def __init__(self, cell, goal, parent):
        self.cell = cell
        self.cum_cost = (parent.cum_cost + self.cell.distance(parent.cell) if parent else 0)
        self.min_cost = self.cum_cost + self.cell.distance(goal)
        self.parent = parent

    def __cmp__(self, other):
        return cmp(self.min_cost, other.min_cost)

class Cell:
    """Class representing a cell in grid world"""
    def __init__(self, location, celltype):
        # Cell coordinates
        self.location = location
        self.celltype = celltype
        self.neighbors = []

    def add_neighbor(self, n):
        if n.is_open():
            self.neighbors.append(n)

    def is_open(self):
        """Whether an agent can be at this cell"""
        return self.celltype == ' '

    def to_world(self):
        """Returns the center of the cell in world coordinates"""
        return tuple(to_world_coordinate(c) for c in self.location)

    def distance(self, other):
        return math.sqrt(sum((c2 - c1) * (c2 - c1) for c1, c2 in zip(self.location, other.location)))

    def world_distance(self, location):
        # Given location is in world coordinates
        return math.sqrt(sum((c2 - c1) * (c2 - c1) for c1, c2 in zip(self.to_world(), location)))

    def closer_point(self, location):
        """Returns a location that is slightly closer to
        the cell than the given location"""
        dist = self.world_distance(location)
        if dist == 0:
            return location
        delta = tuple(0.2 * (c1 - c2) / dist for c1, c2 in zip(self.to_world(), location))
        return tuple(c1 + d for c1, d in zip(location, delta))

    def __str__(self):
        return str(self.location)

    def __repr__(self):
        return str(self)

class Agent:
    """Class representing an agent in the continuous world"""
    def __init__(self, cell, location=None):
        # World coordinates
        self.cell = cell
        self.cell_lock = Lock()
        self.location = location
        if not self.location:
            self.fix_location()

        # Causes initial position to be relayed
        self._waypoints = [self.cell]
        self.waypoint_lock = Lock()

    def set_waypoints(self, waypoints):
        with self.waypoint_lock:
            self._waypoints = waypoints

    def get_waypoints(self):
        with self.waypoint_lock:
            return self._waypoints

    def follow_waypoints(self, callback):
        """Take one step towards next waypoint"""
        waypoints = self.get_waypoints()
        rotationmatrix = [0, 0, 1, 0, 1, 0, -1, 0, 0]
        if len(waypoints) > 0:
            if waypoints[0].world_distance(self.location) < 0.3:
                # Make sure movement is only from center to center
                # to prevent stuck-in-the wall bugs
                callback('MOVE_PLAYER_CELL',
                    ','.join(str(s) for s in
                             (waypoints[0].location[0], self.cell.location[0],
                              waypoints[0].location[1], self.cell.location[1])))
                self.set_cell(waypoints[0])
                # If the waypoints changed from under us, don't try to change them
                if self.get_waypoints() == waypoints:
                    self.set_waypoints(waypoints[1:])
            else:
                newlocation = waypoints[0].closer_point(self.location)
                deltaX = self.location[0] - newlocation[0]
                deltaZ = self.location[1] - newlocation[1]
                angle = math.atan2(deltaX, deltaZ)
                # rotationmatrix = [1,0,0,0,math.cos(angle),1-math.sin(angle),
                # 0,math.sin(angle),math.cos(angle)]

                rotationmatrix = [math.cos(angle), 0, math.sin(angle), 0, 1, 0,
                                  - 1 * math.sin(angle), 0, math.cos(angle)]
                self.location = newlocation
            # Ok, here's where I need to rotate Jr in the direction he is moving
            callback('PLAYER_MOVE_3D', ','.join(str(s) for s in [self.location[0], 0, self.location[1]] + rotationmatrix))

    def fix_location(self):
        """Moves the agent to the center of its cell"""
        self.location = self.cell.to_world()

    def set_cell(self, cell):
        """Sets the agent's cell and the location to the center of it"""
        # TODO: Make reads of cell coherent as well
        with self.cell_lock:
            self.cell = cell
            self.fix_location()

    def plan_path(self, goal):
        """A* search"""
        explored = set()
        frontier = [Node(self.cell, goal, None)]
        print "Planning path from {} to {}...".format(self.cell.location, goal.location)
        while True:
            try:
                current = heapq.heappop(frontier)
            except IndexError:
                break
            if current.cell.location in explored:
                continue
            #print 'Exploring: %s (%d)' % (str(current.cell), current.min_cost)
            explored.add(current.cell.location)
            if current.cell is goal:
                waypoints = []
                while current is not None:
                    waypoints.append(current.cell)
                    current = current.parent
                waypoints.reverse()
                self.set_waypoints(waypoints)
                print "Found path"
                return True
            for n in current.cell.neighbors:
                if n.location not in explored:
                    heapq.heappush(frontier, Node(n, goal, current))
        print "Error: could not find path"
        return False

class Room:
    """Class representing rooms in map"""
    def __init__(self, coords, name):
        self.coords = coords
        self.name = name
        self.center = (coords[0] + (coords[2] - coords[0]) / 2, coords[1] + (coords[3] - coords[1]) / 2)

    def __contains__(self, location):
        """Checks if x,z coordinates in a room"""
        x_coords = location[0] >= self.coords[0] and location[0] <= self.coords[2]
        y_coords = location[1] >= self.coords[1] and location[1] <= self.coords[3]
        return x_coords and y_coords


class GameEnvironment:
    """Class representing the game environment"""
    def __init__(self, env):
        self.grid = []
        self.rooms = {}
        self.objects = []
        self.object_positions = {}
        bombs = 0
        badguys = 0
        hostages = 0
        for i, line in enumerate(env.split(ROW_DELIMITER)):
            if line.startswith('r') or line.startswith('h'):
                room_data = line.split(":")
                coords = room_data[0].split(" ")
                coords = coords[1:]
                coords = [int(item) for item in coords]
                # Repack coords from [z1, x1, z2, x2] to [x1, z1, x2, z2]
                coords = [coords[1], coords[0], coords[3], coords[2]]
                name = room_data[1]
                self.rooms[name] = (Room(coords, name))
            elif len(line.strip()) > 0:
                self.grid.append([])
                for j, c in enumerate(line):
                    new_cell = Cell((i, j), to_simple_cell(c))
                    self.grid[-1].append(new_cell)
                    if c == 'C':
                        self.cmdr = Agent(new_cell)
                    elif c == 'J':
                        self.jr = Agent(new_cell)
                    elif c == 'E':
                        badguys = badguys + 1
                        new_badguy = "badguy" + str(badguys)
                        self.objects.append(new_badguy)
                        self.object_positions[new_badguy] = (i, j)
                    elif c == '*':
                        bombs = bombs + 1
                        new_bomb = "bomb" + str(bombs)
                        self.objects.append(new_bomb)
                        self.object_positions[new_bomb] = (i, j)
                    elif c == 'H':
                        hostages = hostages + 1
                        new_hostage = "hostage" + str(hostages)
                        self.objects.append(new_hostage)
                        self.object_positions[new_hostage] = (i, j)

        for i, row in enumerate(self.grid):
            for j, cell in enumerate(row):
                if i > 0:
                    cell.add_neighbor(self.grid[i - 1][j])
                if i < len(self.grid) - 1:
                    cell.add_neighbor(self.grid[i + 1][j])
                if j > 0:
                    cell.add_neighbor(self.grid[i][j - 1])
                if j < len(self.grid) - 1:
                    cell.add_neighbor(self.grid[i][j + 1])
                print "Cell ({}, {}) has neighbors: {}".format(i, j, cell.neighbors)
        print 'Created environment:'
        print str(self)

    def update_cmdr(self, location):
        """Updates commander's location"""
        self.cmdr.set_cell(self.grid[location[0]][location[1]])

    def update_jr(self, location):
        """Update junior's location"""
        self.jr.set_cell(self.grid[location[0]][location[1]])

    def cell_contents(self, cell):
        """Returns either the cell if empty or its contents"""
        if cell is self.cmdr.cell:
            return 'C'
        elif cell is self.jr.cell:
            return 'J'
        else:
            return cell.celltype

    def __str__(self):
        return '\n'.join(''.join(self.cell_contents(cell) for cell in row) \
                             for row in self.grid)

def to_cell_coordinate(c):
    """Continuous world to grid world"""
    return int(c / 4)

def to_world_coordinate(c):
    """Grid world to continuous world"""
    return c * 4

def to_simple_cell(c):
    """Returns a cell's type as either dead or open space"""
    if c in ('|', '-', '+', '1', '2', '3', '4'):
        # Dead space
        return '-'
    else:
        # Open space
        return ' '
