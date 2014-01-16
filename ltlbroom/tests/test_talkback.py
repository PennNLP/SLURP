"""Test talkback."""

# Copyright (C) 2014 Constantine Lignos
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

from semantics.new_structures import Command, ObjectEntity
from semantics.lexical_constants import *
from ltlbroom.talkback import *


class TestTalkback(unittest.TestCase):
    """Test talkback."""

    def setUp(self):
        self.interpreter = ResponseInterpreter()

    def test_abort(self):
        """Test talkback for a crash."""
        response = CommandResponse(None, AbortError())
        self.assertEqual(self.interpreter.interpret(response),
                         ResponseInterpreter.CRASH)

    def test_misunderstand(self):
        """Test talkback for failure to extract a command."""
        response = CommandResponse(None, None)
        self.assertEqual(self.interpreter.interpret(response),
                         ResponseInterpreter.MISUNDERSTAND)

    def test_unknown(self):
        """Test talkback for an action we cannot perform."""
        unknown = "eat"
        response = CommandResponse(Command(None, None, None, None, None, None, unknown),
                                   UnknownActionError())
        self.assertEqual(self.interpreter.interpret(response),
                         ResponseInterpreter.CANNOT.format(unknown))

    def test_bad_region(self):
        """Test talkback for a command with a bad region."""
        location = ObjectEntity("aslkjal")
        response = CommandResponse(Command(None, None, None, location, None, None, GO_ACTION),
                                   NoSuchLocationError(location.name))
        self.assertEqual(self.interpreter.interpret(response),
                         ResponseInterpreter.NO_LOCATION.format(location.name))

    def test_go(self):
        """Test talkback for a go command."""
        location = ObjectEntity("hallway")
        response = CommandResponse(Command(None, None, None, location, None, None, GO_ACTION),
                                   None)
        self.assertEqual(
            self.interpreter.interpret(response),
            ResponseInterpreter.GOTIT_LONG.format(
                GO_ACTION, ResponseInterpreter.PREP_REGION.format("to", location.name)) + '.')

    def test_patrol(self):
        """Test talkback for a patrol command."""
        location = ObjectEntity("hallway")
        response = CommandResponse(Command(None, None, None, location, None, None, PATROL_ACTION),
                                   None)
        self.assertEqual(
            self.interpreter.interpret(response),
            ResponseInterpreter.GOTIT_LONG.format(
                PATROL_ACTION, ResponseInterpreter.REGION.format(location.name)) + '.')

    def test_defuse(self):
        """Test talkback for a defuse command."""
        bomb = "bomb"
        command = Command(None, ObjectEntity(DEFUSE_ACTION), None, None, None, None, ACTIVATE_ACTION,
                          Command(None, ObjectEntity(bomb), None, None, None, None, SEE_ACTION))
        response = CommandResponse(command, None)
        self.assertEqual(self.interpreter.interpret(response),
                         ResponseInterpreter.CONDITION.format(bomb) + ", " +
                         ResponseInterpreter.GOTIT.format(DEFUSE_ACTION) + '.')

    def test_location(self):
        """Test talkback for a location command."""
        location = ObjectEntity("hallway")
        command = Command(None, ObjectEntity(DEFUSE_ACTION), None, location, None, None, ACTIVATE_ACTION)
        response = CommandResponse(command, None)
        self.assertEqual(
            self.interpreter.interpret(response),
            ResponseInterpreter.GOTIT_LONG.format(
                repr(DEFUSE_ACTION), ResponseInterpreter.PREP_REGION.format("in", location.name)) + '.')


if __name__ == '__main__':
    unittest.main()
