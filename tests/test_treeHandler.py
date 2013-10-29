'''
Created on Oct 17, 2013

@author: taylor
'''
from nltk import Tree
import sys
from semantics.treeHandler import TreeHandler

import unittest

def main():
    tth = TestTreeHandler()
    tth.test_main_pos_phrasepath()
    
class TestTreeHandler(unittest.TestCase):
    '''Test the TreeHandler.'''
    
    def setUp(self):
        self.th = TreeHandler()
        self.exdict = {"carry_from_to": {
                                                 "sent" : "Carry the meals from the kitchen to the rooms." ,
                                                 "tree" : Tree('''(S\n  (NP-SBJ-A (-NONE- *))\n  (VP\n    (VB Carry)\n    (NP-A (DT the) (NNS meals))\n    (PP-CLR (IN from) (NP-A (DT the) (NN kitchen)))\n    (PP-CLR (TO to) (NP-A (DT the) (NNS rooms))))\n  (. .))'''),
                                                 }
                               }
        
    def test_main_pos_phrasepath(self):
        tree = self.exdict["carry_from_to"]["tree"]
        pos = "VB"
        cursor = [-1]
        path = self.th.get_main_pos_phrasepath(tree,pos,-1,cursor)
        self.assertEqual(path,[('S', 1), ('VP', 0), ('VB', 0)])
            
if __name__=="__main__":
    #main()
    unittest.main()
                
