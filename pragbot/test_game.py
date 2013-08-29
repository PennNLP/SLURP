"""Test the Pragbot game world."""

# Copyright (C) 2013 Israel Geselowitz and Constantine Lignos
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import unittest

from pragbot.game import GameEnvironment

GAME_ENV = \
"""         +----------+-----------+          ;         |          |           |          ;         |          |   E       |          ;         |          |           |          ;+--------+- -----4--+--------- -+---------+;|H       3                      |         |;|  *         +------+-------+             |;|   H  E |   1 *H*H |       |   |         |;|        |          |           |         |;+--------+   +------+-------+   +---------+;|  *     4   |E  E  |       4   1         |;|H   E   |          |           |         |;|            +------+-------+             |;|        |   2    E |   *   |   |         |;+--------+   |      |   H   |   +---------+;|        |   |      |       |   2 *       |;|        1   +-- ---+--- -3-+   | * E     |;|                                 H       |;|        |                      |         |;+--------+---------  -----------+---------+;              |    XX     |                ;              |           |                ;              |  C     J  |                ;              +-----------+                ;r 1 5 8 8:office;r 1 10 8 13:classroom;r 1 15 8 18:bathroom;r 10 1 19 3:kitchen;r 21 1 31 3:conservatory;r 33 5 41 8:dining_room;r 33 10 41 13:computer_room;r 33 15 41 18:billiard_room;r 14 7 19 8:annex;r 21 7 27 8:ballroom;r 14 10 19 11:library;r 21 10 27 11:lounge;r 14 13 19 15:cellar;r 21 13 27 15:study;h 15 20 25 22:entrance;h 10 5 31 5:hallway1;h 10 17 31 18:hallway2;h 10 6 12 16:hallway3;h 29 6 31 16:hallway4;"""

class TestPathPlan(unittest.TestCase):

    def setUp(self):
        self.ge = GameEnvironment(GAME_ENV)

    def test_connectivity(self):
        """Test that the center of all rooms are reachable from each other."""
        jr = self.ge.jr
        rooms = self.ge.rooms
        for room1 in rooms.values():
            room1_center = self.ge.get_cell(room1.center)
            for room2 in rooms.values():
                room2_center = self.ge.get_cell(room2.center)
                # Set JR's location to one center and move to the other
                jr.set_cell(room1_center)
                self.assertTrue(jr.plan_path(room2_center),
                                "Could not reach {} from {}".format(room2.name, room1.name))

class TestGameEnvironment(unittest.TestCase):

    def test_init(self):
        """Test that a game environment can be loaded."""
        self.ge = GameEnvironment(GAME_ENV)


def setup_logging():
    """Set up logging so it looks more like when we are running."""
    import logging
    logging.getLogger().setLevel(logging.WARNING)


if __name__ == '__main__':
    setup_logging()
    unittest.main()
