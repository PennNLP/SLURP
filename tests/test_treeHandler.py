'''
Created on Oct 17, 2013

@author: taylor
'''
from nltk import Tree
import sys
from semantics.treeHandler import TreeHandler
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
                                                 }
                               }
        
        self.crazyframes = {"SUBPHRASE_PASS": 
              [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('DT','DT','',''),('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],
              "SUBPHRASE_FAIL": 
              [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''),('DT','DT','',''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')]
             }

        
    def test_main_pos_phrasepath(self):
        tree = self.exDict["carry_from_to"]["tree"]
        pos = "VB"
        cursor = [-1]
        path = self.th.get_main_pos_phrasepath(tree,pos,-1,cursor)
        self.assertEqual(path,[('S', 1), ('VP', 0), ('VB', 0)])
        
    def test_cursor(self): 
        #Test to see that cursor works for the treehandler, only difference should be the depth of the determiner       
        matcher = ParseMatcher(0,2)        
        tree = self.exDict["carry_from_to"]["tree"]
        matcher.th.depth_ulid_augment(tree,0)
        key = "SUBPHRASE_PASS"
        frame = self.crazyframes[key]            
        match = matcher.match_frame(frame,tree)
        self.assertEqual(match,{'Destination': Tree('NNS.4', ['rooms']), 'Agent': Tree('-NPNONE-.2', ['*']), 'to towards': Tree('TO.3', ['to']), 'Theme': Tree('NNS.3', ['meals']), 'VERB': Tree('VB.2', ['Carry']), 'DT': Tree('DT.3', ['the'])})
        key = "SUBPHRASE_FAIL"
        frame = self.crazyframes[key]            
        match = matcher.match_frame(frame,tree)
        self.assertEqual(match,{'Destination': Tree('NNS.4', ['rooms']), 'Agent': Tree('-NPNONE-.2', ['*']), 'to towards': Tree('TO.3', ['to']), 'Theme': Tree('NNS.3', ['meals']), 'VERB': Tree('VB.2', ['Carry']), 'DT': Tree('DT.4', ['the'])})
        
            
if __name__=="__main__":
    #main()
    unittest.main()
                
