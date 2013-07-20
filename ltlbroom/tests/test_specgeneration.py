"""Test components of spec generation."""

import random
import unittest

from ltlbroom.specgeneration import SpecGenerator

class TestSpecGenerator(unittest.TestCase):

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
        self.assertEqual(syl, [
            '([](((s.camera & !s.radio) | (s.radio & !s.camera) '
            '| (!s.camera & !s.radio))))'
            ])
        
    def test_activate(self):
        """Test a simple activate command."""
        self.props = ['camera']
        text = "Activate your camera."""
        enl, syl = self.lines_from_gen(text)
        self.assertEqual(enl, [])
        self.assertEqual(syl, ['([](s.camera))'])

    def test_activate_conditional(self):
        """Test a conditional activate command."""
        self.props = ['camera']
        text = "Activate your camera in the hallway."""
        enl, syl = self.lines_from_gen(text)
        self.assertEqual(enl, [])
        self.assertEqual(syl, ['([]((s.hallway -> s.camera)))'])

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
