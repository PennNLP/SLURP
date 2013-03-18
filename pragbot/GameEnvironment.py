#!/usr/bin/env python

ROW_DELIMITER = ';'
DEFAULT_MAP = 'pragbot/Maps/ScenarioEnv.txt'

class Cell:
    def __init__(self, location, type):
        # Cell coordinates
        self.location = location
        self.type = type

    def is_open(self):
        return self.type == ' '

    def to_world(self):
        return tuple(to_world_coordinate(c) for c in self.location)

class Agent:
    def __init__(self, cell, location=None):
        # World coordinates
        self.cell = cell
        self.location = location
        if not self.location:
            self.fix_location

    def fix_location(self):
        self.location = self.cell.to_world()

    def set_cell(self, cell):
        self.cell = cell
        self.fix_location()

class GameEnvironment:
    def __init__(self, env_file=DEFAULT_MAP):
        self.grid = []
        self.rooms = []
        self.original_lines = []
        with open(env_file) as f:
            for i,line in enumerate(f.readlines()):
                line = line.rstrip('\r\n')
                self.original_lines.append(line)
                if line.startswith('r'):
                    self.rooms.append(line)
                else:
                    self.grid.append([])
                    for j,c in enumerate(line):
                        new_cell = Cell((i,j), to_simple_cell(c))
                        self.grid[-1].append(new_cell)
                        if c == 'C':
                            self.cmdr = Agent(new_cell)
                        elif c== 'J':
                            self.jr = Agent(new_cell)

    def to_fps_string(self):
        return ROW_DELIMITER.join(self.original_lines)

    def update_cmdr(self, location):
        self.cmdr.set_cell(self.grid[location[0]][location[1]])
        
    def update_jr(self, location):
        self.jr.set_cell(self.grid[location[0]][location[1]])

    def cell_contents(self, cell):
        if cell == self.cmdr.cell:
            return 'C'
        elif cell == self.jr.cell:
            return 'J'
        else:
            return cell.type

    def __str__(self):
        return '\n'.join(''.join(self.cell_contents(cell) for cell in row) for row in self.grid)

def to_cell_coordinate(c):
    return int(c/4)

def to_world_coordinate(c):
    return c*4

def to_simple_cell(c):
    if c in ('|', '-', '+', '%', '$'):
        # Dead space
        return '-'
    else:
        # Open space
        return ' '

# For testing purposes only
if __name__ == '__main__':
    ge = GameEnvironment()
    print ge.to_string()
    print ge.to_fps_string()
