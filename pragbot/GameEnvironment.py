#!/usr/bin/env python

ROW_DELIMITER = ';'
DEFAULT_MAP = 'pragbot/Maps/ScenarioEnv.txt'

class GameEnvironment:
    def __init__(self, env_file=DEFAULT_MAP):
        self.grid = [] # List of strings
        self.rooms = []
        with open(env_file) as f:
            for line in f.readlines():
                line = line.rstrip('\r\n')
                if line.startswith('r'):
                    self.rooms.append(line)
                else:
                    self.grid.append(line)


    def to_fps_string(self):
        return ROW_DELIMITER.join(self.grid + self.rooms)

    def to_string(self):
        return ROW_DELIMITER.join(''.join(to_simple_cell(c) for c in row) for row in self.grid) + 'NEW_SECTION'


def to_simple_cell(c):
    if c == 'b':
        # Barrier
        return b
    elif c in ('|', '-', '+', '%', '$'):
        # Dead space
        return '-'
    else:
        # Open space
        return ' '

if __name__ == '__main__':
    ge = GameEnvironment()
    print ge.to_string()
    print ge.to_fps_string()
