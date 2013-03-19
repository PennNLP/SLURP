#!/usr/bin/env python

ROW_DELIMITER = ';'
DEFAULT_MAP = 'pragbot/Maps/ScenarioEnv.txt'

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

    def __str__(self):
        return str(self.location)

    def __repr__(self):
        return str(self)

class Agent:
    """Class representing an agent in the continuous world"""
    def __init__(self, cell, location=None):
        # World coordinates
        self.cell = cell
        self.location = location
        if not self.location:
            self.fix_location()
        self.waypoints = []

    def fix_location(self):
        """Moves the agent to the center of its cell"""
        self.location = self.cell.to_world()

    def set_cell(self, cell):
        """Sets the agent's cell and the location to the center of it"""
        self.cell = cell
        self.fix_location()

    def plan_path(self, goal):
        """Breadth-first path search"""
        # TODO: convert this to A*
        parent_dict = {self.cell: None}
        explored = set()
        frontier = [self.cell]
        while len(frontier) != 0:
            current = frontier.pop(0)
            if current is goal:
                self.waypoints = []
                while current is not None:
                    self.waypoints.append(current)
                    current = parent_dict[current]
                self.waypoints.reverse()
                return True
            explored.add(current)
            for n in current.neighbors:
                if n not in explored:
                    parent_dict[n] = current
                    frontier.append(n)
        return False

class GameEnvironment:
    """Class representing the game environment"""
    def __init__(self, env_file=DEFAULT_MAP):
        self.grid = []
        self.rooms = []
        self.original_lines = []
        with open(env_file) as f:
            for i, line in enumerate(f.readlines()):
                line = line.rstrip('\r\n')
                self.original_lines.append(line)
                if line.startswith('r'):
                    self.rooms.append(line)
                else:
                    self.grid.append([])
                    for j, c in enumerate(line):
                        new_cell = Cell((i, j), to_simple_cell(c))
                        self.grid[-1].append(new_cell)
                        if c == 'C':
                            self.cmdr = Agent(new_cell)
                        elif c == 'J':
                            self.jr = Agent(new_cell)
        for i, row in enumerate(self.grid):
            for j, cell in enumerate(row):
                if i > 1:
                    cell.add_neighbor(self.grid[i-1][j])
                if i < len(self.grid) - 1:
                    cell.add_neighbor(self.grid[i+1][j])
                if j > 1:
                    cell.add_neighbor(self.grid[i][j-1])
                if j < len(self.grid) - 1:
                    cell.add_neighbor(self.grid[i][j+1])
    def to_fps_string(self):
        """Returns the fps_string as understood by the client"""
        return ROW_DELIMITER.join(self.original_lines)

    def update_cmdr(self, location):
        """Updates commander's location"""
        self.cmdr.set_cell(self.grid[location[0]][location[1]])
        
    def update_jr(self, location):
        """Update junior's location"""
        self.jr.set_cell(self.grid[location[0]][location[1]])

    def cell_contents(self, cell):
        """Returns either the cell if empty or its contents"""
        if cell == self.cmdr.cell:
            return 'C'
        elif cell == self.jr.cell:
            return 'J'
        else:
            return cell.celltype

    def __str__(self):
        return '\n'.join(''.join(self.cell_contents(cell) for cell in row) \
                             for row in self.grid)

def to_cell_coordinate(c):
    """Continuous world to grid world"""
    return int(c/4)

def to_world_coordinate(c):
    """Grid world to continuous world"""
    return c*4

def to_simple_cell(c):
    """Returns a cell's type as either dead or open space"""
    if c in ('|', '-', '+', '%', '$'):
        # Dead space
        return '-'
    else:
        # Open space
        return ' '

# For testing purposes only
if __name__ == '__main__':
    ge = GameEnvironment()
    print str(ge)
    ge.jr.plan_path(ge.cmdr.cell)
    print ge.jr.waypoints
