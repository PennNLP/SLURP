'''
Created on Oct 17, 2013

@author: taylor
'''
from nltk import Tree
import sys
from semantics.matching import ParseMatcher
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
                                   "correct_frame" :   [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')] 
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
     
    def test_pp(self):        
        tree1 = self.correct
        tree2 = self.incorrect
        matcher = ParseMatcher(0,2)
        matcher.th.depth_ulid_augment(tree1,0)
        matcher.th.depth_ulid_augment(tree2,0)
        print 'Tests for correct and incorrect PP attachment for trees'
        print 'Correct attachment:'
        print tree1
        print 'Incorrect attachment:',
        print tree2                        
        for frame in self.pp_attach_framelist:
            print "For frame(",frame,"):"
            print "Correct tree frame match:"
            match = matcher.match_frame(frame,tree1)
            print "Incorrect tree frame match:"
            match = matcher.match_frame(frame, tree2)
                        

            
    def test_strict(self):
        matcher = ParseMatcher()
        matcher.th.depth_ulid_augment(self.correct,0)
            
    def test_theme_np_pp(self):
        '''An NP inside of a PP should not be allowed to be a theme.'''
        key = "defuse_in"
        tree = self.exDict[key]["tree"]
        wrongFrame = self.exDict[key]["wrong_frame"]
        frames = extract_frames_from_parse(tree,verbose=True)
        self.assertEqual(len(frames),1)
        self.assertNotEqual(frames[0],wrongFrame)
        
        
        
        
        
        
            
if __name__=="__main__":
    unittest.main()
                
