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

from time import sleep


from pipelinehost import PipelineClient


def stress():
    """Stress the pipeline."""
    parser = PipelineClient()
    while True:
        try:
            parser.parse("This is the tremendously violent stress test of doom.")
            sleep(.1)
        except KeyboardInterrupt:
            break
    
    parser.close()

if __name__ == "__main__":
    stress()
