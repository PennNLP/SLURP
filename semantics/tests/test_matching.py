'''
Created on Oct 17, 2013

@author: taylor
'''
from nltk import Tree
import sys
from semantics.matching import ParseMatcher
from semantics.treehandler import TreeHandler
from pipelinehost import PipelineClient
from semantics.parsing import extract_frames_from_parse

import unittest

class exampelPPAttachment(unittest.TestCase):
    '''    Syntax examples for the two sentences:
            Carry the meals from the kitchen to the rooms.
            *Carry the meals from the kitchen to the cafeteria. -> yields an incorrect syntax parse
        Used to develop strict verbframe matching and pp attachment.
    '''
    def setUp(self):
        self.exDict = {"carry_from_to" :{"sent" : "Carry the meals from the kitchen to the rooms.",
                                   "tree" :  Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Carry']), Tree('NP-A', [Tree('DT', ['the']), Tree('NNS', ['meals'])]), Tree('PP-CLR', [Tree('IN', ['from']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['kitchen'])])]), Tree('PP-CLR', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NNS', ['rooms'])])])]), Tree('.', ['.'])]),
                                   "possible_frames": [[('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', '')],
                                                       [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],
                                                       [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],    
                                                       [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', '')]],
                                   "correct_frame" :   [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],
                                   "correct_entry" : {'to towards': Tree('TO.3', ['to']), 'Destination': Tree('NNS.4', ['rooms']), 'Agent': Tree('-NPNONE-.2', ['*']), 'Source': Tree('NN.4', ['kitchen']), 'Theme': Tree('NNS.3', ['meals']), 'VERB': Tree('VB.2', ['Carry'])}                                    
                                   },
                  "carry_from_to_deep_pp":{"sent" : "Carry the meals from the kitchen to the cafeteria.",
                                   "tree" :  Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Carry']), Tree('NP-A', [Tree('DT', ['the']), Tree('NNS', ['meals'])]), Tree('PP-CLR', [Tree('IN', ['from']), Tree('NP-A', [Tree('NP', [Tree('DT', ['the']), Tree('NN', ['kitchen'])]), Tree('PP', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['cafeteria'])])])])])]), Tree('.', ['.'])]),
                                   "possible_frames": [[('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', '')],
                                                       [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],
                                                       [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],    
                                                       [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', '')]],
                                   "correct_frame" :   [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')] 
                                   },           
                "defuse_in"   : {"sent" : "Defuse in the hallway.",
                                 "tree" : Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Defuse']), Tree('PP-CLR', [Tree('IN', ['in']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['hallway'])])])]), Tree('.', ['.'])]),
                                 "wrong_frame" : [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', '')]
                                 }
                  }
        
        self.crazyframes = {"SUBPHRASE_PASS": 
                       [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('DT','DT','',''),('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],
                       "SUBPHRASE_FAIL": 
                       [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''),('DT','DT','',''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')]
                      }
        self.th = TreeHandler()
        self.matcher = ParseMatcher(0,2)
        
    def test_from_to(self):
        tree = self.exDict["carry_from_to"]["tree"]
        possible_frames = self.exDict["carry_from_to"]["possible_frames"]
        correct_entry = self.exDict["carry_from_to"]["correct_entry"]
        self.th.depth_ulid_augment(tree, 0)        
        matches = []
        for frame in possible_frames:
            match = self.matcher.match_frame(frame,tree)
            if match:
                matches.append(match)
        self.assertIn(correct_entry,matches)
     
    def test_theme_np_pp(self):
        '''An NP inside of a PP should not be allowed to be a theme.'''
        key = "defuse_in"
        tree = self.exDict[key]["tree"]
        self.th.depth_ulid_augment(tree, 0)
        wrongFrame = self.exDict[key]["wrong_frame"]
        match = self.matcher.match_frame(wrongFrame, tree)
        self.assertFalse(match)
        
        
        
        
        
        
            
if __name__=="__main__":
    unittest.main()
                
