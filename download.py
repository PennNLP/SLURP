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

import sys
import urllib2
import posixpath
import shutil
from zipfile import ZipFile, BadZipfile

URLS = ("http://www.seas.upenn.edu/~lignos/data/nlpipeline.zip",)

for address in URLS:
    filename = posixpath.basename(address)
    print "Downloading %s..." % address

    try:
        url = urllib2.urlopen(address)
    except urllib2.HTTPError:
        print >> sys.stderr, "Couldn't open URL", address
        continue

    try:
        with open(filename, 'wb') as localfile:
            shutil.copyfileobj(url, localfile)
    except IOError:
        print >> sys.stderr, "Couldn't write file", filename
        continue
    finally:
        url.close()

    print "Unzipping %s..." % filename
    try:
        zfile = ZipFile(filename)
        zfile.extractall()
    except BadZipfile:
        print >> sys.stderr, "Couldn't unzip file", filename
