'''
Created on Oct 17, 2013

@author: taylor
'''
from nltk import Tree
import sys
from semantics.matching import ParseMatcher

def main():
    print 'hello world'
    matcher = ParseMatcher(0,2)
    testList = [exampelPPAttachment().tests]
    for category in testList:
        for test in category:
            test(matcher)
    
class exampelPPAttachment(object):
    '''    Syntax examples for the two sentences:
            Carry the meals from the kitchen to the rooms.
            *Carry the meals from the kitchen to the cafeteria. -> yields an incorrect syntax parse
        Used to develop strict verbframe matching and pp attachment.
    '''
    correct = Tree('''(S\n  (NP-SBJ-A (-NONE- *))\n  (VP\n    (VB Carry)\n    (NP-A (DT the) (NNS meals))\n    (PP-CLR (IN from) (NP-A (DT the) (NN kitchen)))\n    (PP-CLR (TO to) (NP-A (DT the) (NNS rooms))))\n  (. .))''')
    incorrect = Tree('''(S\n  (NP-SBJ-A (-NONE- *))\n  (VP\n    (VB Carry)\n    (NP-A (DT the) (NNS meals))\n    (PP-CLR\n      (IN from)\n      (NP-A\n        (NP (DT the) (NN kitchen))\n        (PP (TO to) (NP-A (DT the) (NN cafeteria))))))\n  (. .))''')
    strictWillFail = Tree('''(S\n  (NP-SBJ-A (-NONE- *))\n  (VP\n    (VB Carry)\n    (NP-A (DT the) (NNS meals))\n    (PP-CLR\n      (IN from)\n      (NP-A\n        (NP (DT the) (NN kitchen))\n        (PP (IN in) (NP-A (DT the) (NN cafeteria))))))\n  (. .))''') 

    pp_attach_framelist = [
                  [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', '')],
                  [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],
                  [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],    
                  [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', '')],   
                  [('NP', 'Theme', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Value', '', '')],    
                  [('NP', 'Theme', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Beneficiary', '', ''), ('NP', 'Value', '', '')]                
                ]
    
    strict = {"STRICT_FAIL": {"tree" : Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Carry']), Tree('NP-A', [Tree('DT', ['the']), Tree('NNS', ['meals'])]), Tree('PP-CLR', [Tree('IN', ['from']), Tree('NP-A', [Tree('NP', [Tree('DT', ['the']), Tree('NN', ['kitchen'])]), Tree('PP', [Tree('IN', ['in']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['cafeteria'])])])])])]), Tree('.', ['.'])]),
                                "frame" : [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Destination', '', '')]
                            }
              }
    
    crazyframes = {"SUBPHRASE_PASS": 
                   [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('DT','DT','',''),('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],
                   "SUBPHRASE_FAIL": 
                   [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''),('DT','DT','',''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')]
                  }
     
    def __init__(self):
        tests = [self.pp_test,self.strict_test,self.cursor_test,]
        #tests = [self.pp_test]
        #tests = [self.strict_test]
        self.tests = tests
    
    def pp_test(self,matcher):        
        tree1 = self.correct
        tree2 = self.incorrect
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
                        
    def cursor_test(self,matcher):
        matcher.th.depth_ulid_augment(self.correct,0)
        tree = self.correct
        print "Tests for tree: "
        print tree
        for key in self.crazyframes:
            print "Running test(",key,")"
            frame = self.crazyframes[key]            
            match = matcher.match_frame(frame,tree)
            
    def strict_test(self,matcher):
        matcher.th.depth_ulid_augment(self.correct,0)
        d = self.strict
        for test in d:
            tree = d[test]["tree"]
            frame = d[test]["frame"]
            match = matcher.match_frame(frame,tree)
            
if __name__=="__main__":
    main()
                
