#!/usr/bin/env python
"""Download data files needed by SLURP."""

# Copyright (C) 2012-2013 Constantine Lignos
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
import gzip

from datamanager import download, unzip


# Assume that download.py and pennpipeline.py are located in the same
# directory
PIPELINE_NAME = "SUBTLEPipeline-master"
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
PIPELINE_URL = "https://github.com/PennNLP/SUBTLEPipeline/archive/master.zip"
FILENAME = os.path.join(ROOT_DIR, PIPELINE_NAME + ".zip")

download(PIPELINE_URL, FILENAME)
unzip(FILENAME, ROOT_DIR)

# Now we need to additionally unzip the model file contained in the pipeline
print "Decompressing parser model file..."
model_gz = gzip.open(os.path.join(ROOT_DIR, PIPELINE_NAME, "models", "wsjall.obj.gz"))
open(os.path.join(ROOT_DIR, PIPELINE_NAME, "models", "wsjall.obj"), 'wb').write(model_gz.read())
