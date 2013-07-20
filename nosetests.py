#!/usr/bin/env python
"""Run all nose tests."""

import sys
import os

import nose

from pipelinehost import PipelineHost

# Run the pipelinehost in advance just in case. If there's already a
# pipeline running, this will just fail without causing any problems.
try:
    print "Launching PipelineHost..."
    pipeline = PipelineHost(local=True)
except IOError:
    print "PipelineHost appears to already be running."

# Put the root of SLURP on the path, just to be safe
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
nose.main()
