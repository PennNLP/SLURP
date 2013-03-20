"""
Functions for managing data.
Constantine Lignos
March 2012
"""

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

import urllib2
import posixpath
import zipfile


def download(url, path=None):
    """Download a url, save under the same filename or the specified path, and return the path."""
    print "Downloading %s..." % url
    try:
        url_file = urllib2.urlopen(url)
    except urllib2.HTTPError:
        raise IOError("Couldn't open URL %s." % repr(url))

    # Use the provided path, or default to the basename
    filename = path if path else posixpath.basename(url)
    try:
        localFile = open(filename, 'wb')
        localFile.write(url_file.read())
        localFile.close()
    except IOError:
        raise IOError("Couldn't write filename %s." % repr(filename))

    return filename


def unzip(filepath, destpath='.'):
    """Unzip a file."""
    print "Unzipping %s..." % repr(filepath)
    try:
        zfile = zipfile.ZipFile(filepath, 'r')
    except (IOError, zipfile.BadZipfile):
        raise IOError("The zip file %s could not be opened." % repr(filepath))

    zfile.extractall(destpath)
