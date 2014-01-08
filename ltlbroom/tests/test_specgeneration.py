"""Test components of spec generation."""

# Copyright (C) 2011-2013 Constantine Lignos
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

from ltlbroom.specgeneration import SpecGenerator


class TestSpecGenerator(unittest.TestCase):
    """Test spec generation."""

    def setUp(self):
        self.specgen = SpecGenerator()
        self.sensors = []
        self.regions = ['hallway']
        self.props = []
        self.tag_dict = {}

    def test_go(self):
        """Test a basic go command."""
        text = "Go to the hallway."
        enl, syl = self.lines_from_gen(text)
        self.assertEqual(enl, [])
        self.assertEqual(syl, [
            '!s.mem_visit_hallway',
            '([]((next(s.mem_visit_hallway) <-> (s.mem_visit_hallway | next(s.hallway)))))',
            '([]<>(s.mem_visit_hallway))'
            ])

    def test_go_nonexistent(self):
        """Test going to a nonexistent room."""
        text = "Go to the kitchen."
        results = self.lines_from_gen(text)
        self.assertFalse(results[0])

    def test_actuator_mutex(self):
        """Test the actuator mutex."""
        self.props = ['camera', 'radio']
        text = ""
        enl, syl = self.lines_from_gen(text)
        self.assertEqual(enl, [])
        # Assumes mutex is the last sys line
        self.assertEqual(syl[-1], '([](((s.camera & !s.radio) | (s.radio & !s.camera) '
                         '| (!s.camera & !s.radio))))')

    def test_activate(self):
        """Test a simple activate command."""
        self.props = ['camera']
        text = "Activate your camera."
        enl, syl = self.lines_from_gen(text)
        self.assertEqual(enl, [])
        self.assertEqual(syl,
            ['([](((!s.camera & next(s.camera)) -> STAY_THERE)))', '([](next(s.camera)))'])

    def test_activate_location(self):
        """Test an activate command restricted to a location."""
        self.props = ['camera']
        text = "Activate your camera in the hallway."
        enl, syl = self.lines_from_gen(text)
        self.assertEqual(enl, [])
        # Assume actual safety is last line
        self.assertEqual(syl[-1], '([]((next(s.hallway) -> next(s.camera))))')

    def test_activate_conditional1(self):
        """Test an activate command restricted by a preceding conditional."""
        self.props = ['camera']
        self.sensors = ['bomb']
        text = "If you see a bomb, activate your camera."
        enl, syl = self.lines_from_gen(text)
        self.assertEqual(enl, [])
        # Assume actual safety is last line
        self.assertEqual(syl[-1], '([]((next(s.react_bomb) -> next(s.camera))))')

    def test_actuate_all(self):
        """Test an actuation command on 'all'."""
        self.props = ['defuse']
        self.sensors = ['bomb']
        text = "Defuse all bombs."
        enl, syl = self.lines_from_gen(text)
        self.assertEqual(enl, [])
        # Assume actual safety is last line
        self.assertEqual(syl[-1], '([]((next(s.react_bomb) -> next(s.defuse))))')

    def test_actuate_the(self):
        """Test an actuation command on 'the'."""
        self.props = ['defuse']
        self.sensors = ['bomb']
        text = "Defuse the bomb."
        enl, syl = self.lines_from_gen(text)
        self.assertEqual(enl, [])
        # Assume actual safety is last line
        self.assertEqual(syl[-1], '([]((next(s.react_bomb) -> next(s.defuse))))')

    def lines_from_gen(self, text):
        """Return [env_lines, sys_lines] from a default generate call."""
        return self.get_from_gen(text, 0, 2)

    def results_from_gen(self, text):
        """Return responses from a default generate call."""
        return self.get_from_gen(text, 4)

    def get_from_gen(self, text, start, end=None):
        """Return the start-th or start:end element(s) of the return from generate."""
        result = self.specgen.generate(text, self.sensors, self.regions,
          self.props, self.tag_dict, verbose=False)
        if end:
            return result[start:end]
        else:
            return result[start]


if __name__ == '__main__':
    unittest.main()
