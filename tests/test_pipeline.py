"""Test components of spec generation."""

import unittest

from pipelinehost import PipelineClient


class TestPipeline(unittest.TestCase):
    """Test the the pipeline."""

    def setUp(self):
        self.client = PipelineClient()

    def test_parse(self):
        """Test that a simple sentence can be parsed."""
        parse = self.client.parse("This is a test.")
        self.assertEqual(parse,
            "(S  (NP-SBJ-A (DT This)) (VP (VBZ is) (NP-PRD-A (DT a) (NN test)))(. .))")


if __name__ == '__main__':
    unittest.main()
