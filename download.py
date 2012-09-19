#!/usr/bin/env python
"""Download data files needed by SLURP."""

# Copyright (C) 2012 Constantine Lignos
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

import os

from datamanager import download, unzip

# Assume that download.py and pennpipeline.py are located in the same
# directory
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
PIPELINE_URL = "http://www.seas.upenn.edu/~lignos/data/nlpipeline.zip"
FILENAME = os.path.join(ROOT_DIR, "nlpipeline.zip")

download(PIPELINE_URL, FILENAME)
unzip(FILENAME, ROOT_DIR)
