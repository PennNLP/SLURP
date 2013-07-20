#!/usr/bin/env python
"""Stress test for the pipelinehost."""

# Copyright (C) 2013 Constantine Lignos
#
# SLURP is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# SLURP is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with SLURP.  If not, see <http://www.gnu.org/licenses/>.

import random
import string
from multiprocessing import Process

from pipelinehost import PipelineClient


def stress():
    """Stress the pipeline."""
    try:
        while True:
            parser = PipelineClient()
            word = ''.join(random.choice(string.ascii_letters) for _ in range(10))
            parser.parse("This is the tremendously violent stress test of {}.".format(word))
            parser.close()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    procs = []
    print "Starting processes..."
    for _ in range(10):
        proc = Process(target=stress)
        proc.daemon = True
        proc.start()
        procs.append(proc)
    
    print "Running..."
    # Join on an arbitrary process
    try:
        procs[0].join()
    except KeyboardInterrupt:
        # Just exit
        pass

    print "Exited..."
