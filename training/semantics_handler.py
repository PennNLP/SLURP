"""
A handler for parsing the semantics of a sentence for basic training.
"""

# Copyright (C) 2013 Taylor Turpen
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

from pipelinehost import PipelineClient
from semantics.tree import Tree
from semantics.parsing import (extract_frames_from_parse, create_semantic_structures)

class SemanticsHandler(object):

    def __init__(self):
        self.client = PipelineClient(verbose=True)

    def get_semantic_structures(self,sentence):
        parse = self.client.parse(sentence)
        return self.process(parse,verbose=False)

    def process(self,parse,verbose=True):
        """Show the steps of transformation for a parse."""
        # Original parse
        parse_tree = Tree.parse(parse)
    
        frames = extract_frames_from_parse(parse, verbose=verbose)
        if verbose:
            print
            for frame in frames:
                print frame.pprint()
                if frame.condition:
                    print "Condition:"
                    print frame.condition.pprint()
            print
    
        # Bail if no frames matched
        if not frames:
            if verbose: print "No frames matched."
            return
    
        # Extract semantic structures
        semantic_structures = create_semantic_structures(frames)
        if semantic_structures:
            if verbose: print semantic_structures
            return semantic_structures
        else:
            if verbose: print "No semantic structures returned."