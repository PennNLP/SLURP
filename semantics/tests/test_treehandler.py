"""
Tests treehandler functionality of semantics.treehandler
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

from semantics.tree import Tree
from semantics.treehandler import TreeHandler
from semantics.matching import ParseMatcher

import unittest

def main():
    tth = TestTreeHandler()
    tth.test_main_pos_phrasepath()
    
class TestTreeHandler(unittest.TestCase):
    '''Test the TreeHandler.'''
    
    def setUp(self):
        self.th = TreeHandler()
        self.exDict = {"carry_from_to": {
                                                 "sent" : "Carry the meals from the kitchen to the rooms." ,                                                 
                                                 "tree" : Tree('''(S\n  (NP-SBJ-A (-NONE- *))\n  (VP\n    (VB Carry)\n    (NP-A (DT the) (NNS meals))\n    (PP-CLR (IN from) (NP-A (DT the) (NN kitchen)))\n    (PP-CLR (TO to) (NP-A (DT the) (NNS rooms))))\n  (. .))'''),
                                                 "solution" : {'Destination': Tree('NP-A', [Tree('DT', ['the']), Tree('NNS', ['rooms'])]), 'Agent': Tree('NP-SBJ-A', [Tree('-NPNONE-', ['*'])]), 'to towards': Tree('TO', ['to']), 'Theme': Tree('NP-A', [Tree('DT', ['the']), Tree('NNS', ['meals'])]), 'VERB': Tree('VB', ['Carry']), 'DT': Tree('DT', ['the'])}
                                                 }

                               }
        
        self.crazyframes = {"SUBPHRASE_PASS": 
              [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('DT','DT','',''),('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],
              "SUBPHRASE_FAIL": 
              [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''),('DT','DT','',''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')]
             }

        
    def test_main_pos_phrasepath(self):
        """Test that phrase pathing works for treehandler."""
        tree = self.exDict["carry_from_to"]["tree"]
        pos = "VB"
        cursor = [-1]
        path = self.th.get_main_pos_phrasepath(tree,pos,-1,cursor)
        self.assertEqual(path,[('S', 1), ('VP', 0), ('VB', 0)])
        
    def test_cursor(self): 
        """Test that cursor works for treehandler."""
        #Test to see that cursor works for the treehandler, only difference should be the depth of the determiner       
        matcher = ParseMatcher(0,2)        
        tree = self.exDict["carry_from_to"]["tree"]
        solution = self.exDict["carry_from_to"]["solution"]
        matcher.th.depth_ulid_augment(tree,0)
        key = "SUBPHRASE_PASS"
        frame = self.crazyframes[key]            
        match = matcher.match_frame(frame,tree)
        self.assertEqual(match,solution)        
        matcher.th.depth_ulid_augment(tree,0)
        key = "SUBPHRASE_FAIL"
        frame = self.crazyframes[key]            
        match = matcher.match_frame(frame,tree)
        self.assertEqual(match,solution)
        
            
if __name__=="__main__":
    #main()
    unittest.main()
                
