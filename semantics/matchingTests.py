'''
Created on Oct 17, 2013

@author: taylor
'''
from nltk import Tree
import sys

def main():
    print 'hello world'
    
class exampleCoordination(object):
    #Correctly parsed by methodology leads to hardDict's incorrect parses
    easyDict = {"Defuse_VP_VP" : {"sent": "Defuse the bomb and go to the hallway.",
                      "tree": Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VP-A', [Tree('VB', ['Defuse']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['bomb'])])]), Tree('CC', ['and']), Tree('VP-A', [Tree('VB', ['go']), Tree('PP-CLR', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['hallway'])])])])]), Tree('.', ['.'])])
                      },
                }
    exDict = {"Extracts VP, produces S trees with no verb" : {"sent" : "Go and defuse the bomb in the cellar.",
                "tree" : Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Go']), Tree('CC', ['and']), Tree('VB', ['defuse']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['bomb'])]), Tree('PP-MNR', [Tree('IN', ['in']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['cellar'])])])]), Tree('.', ['.'])])
                },
              "Two carrys only one source-dest because of wrongly embedded 'to' pp": {"sent" : "Carry the hostages from the kitchen and the cafeteria to the cellar.",
                                                                "tree" : Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Carry']), Tree('NP-A', [Tree('DT', ['the']), Tree('NNS', ['hostages'])]), Tree('PP-CLR', [Tree('IN', ['from']), Tree('NP-A', [Tree('NP', [Tree('DT', ['the']), Tree('NN', ['kitchen'])]), Tree('CC', ['and']), Tree('NP', [Tree('NP', [Tree('DT', ['the']), Tree('NN', ['cafeteria'])]), Tree('PP', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['cellar'])])])])])])]), Tree('.', ['.'])])
                                                                },
              "VP_NPNP_VP" : { "sent": "Defuse the bomb and the bomb and go to the hallway.",
                              "tree" : Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VP-A', [Tree('VB', ['Defuse']), Tree('NP-A', [Tree('NP', [Tree('DT', ['the']), Tree('NN', ['bomb'])]), Tree('CC', ['and']), Tree('NP', [Tree('DT', ['the']), Tree('NN', ['bomb'])])])]), Tree('CC', ['and']), Tree('VP-A', [Tree('VB', ['go']), Tree('PP-CLR', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['hallway'])])])])]), Tree('.', ['.'])])
                              }
              }
    #Examples the current methods do not correctly parse
    hardDict = { "Carry_NP_NP" : {"sent" : "Carry the hostages from the kitchen and bathroom to the cellar.",
                                  "tree" : Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Carry']), Tree('NP-A', [Tree('DT', ['the']), Tree('NNS', ['hostages'])]), Tree('PP-CLR', [Tree('IN', ['from']), Tree('NP-A', [Tree('NP', [Tree('DT', ['the']), Tree('NN', ['kitchen'])]), Tree('CC', ['and']), Tree('NP', [Tree('NP', [Tree('NN', ['bathroom'])]), Tree('PP', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['cellar'])])])])])])]), Tree('.', ['.'])])                           
                                  },
                "Go_NP_NP" : {"sent": "Go to the cellar and kitchen.",
                              "tree": Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Go']), Tree('PP-CLR', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['cellar']), Tree('CC', ['and']), Tree('NN', ['kitchen'])])])]), Tree('.', ['.'])])
                              },
                "Carry_NP_NP_2" : {"sent": "Carry meals to the cafeteria and lounge.",
                                   "tree": Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Carry']), Tree('NP-A', [Tree('NNS', ['meals'])]), Tree('PP-CLR', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['cafeteria']), Tree('CC', ['and']), Tree('NN', ['lounge'])])])]), Tree('.', ['.'])])
                                   },
                "Carry_NPNP_NPNP" : {"sent" : "Carry meals from the kitchen and cafeteria to the office and lounge.",
                                     #Wrong pp attachment for this tree ("to the office" modifies cafeteria)
                                     "tree" : Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Carry']), Tree('NP-A', [Tree('NNS', ['meals'])]), Tree('PP-CLR', [Tree('IN', ['from']), Tree('NP-A', [Tree('NP', [Tree('DT', ['the']), Tree('NN', ['kitchen'])]), Tree('CC', ['and']), Tree('NP', [Tree('NP', [Tree('NN', ['cafeteria'])]), Tree('PP', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['office'])])])]), Tree('CC', ['and']), Tree('NP', [Tree('NN', ['lounge'])])])])]), Tree('.', ['.'])])
                      },
                "Two carrys only one source-dest because of wrongly embedded 'to' pp": {"sent" : "Carry the hostages from the kitchen and the cafeteria to the cellar.",
                                                                                "tree" : Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Carry']), Tree('NP-A', [Tree('DT', ['the']), Tree('NNS', ['hostages'])]), Tree('PP-CLR', [Tree('IN', ['from']), Tree('NP-A', [Tree('NP', [Tree('DT', ['the']), Tree('NN', ['kitchen'])]), Tree('CC', ['and']), Tree('NP', [Tree('NP', [Tree('DT', ['the']), Tree('NN', ['cafeteria'])]), Tree('PP', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['cellar'])])])])])])]), Tree('.', ['.'])])
                                                                                },
                "Extracts VP, produces S trees with no verb" : {"sent" : "Go and defuse the bomb in the cellar.",
                                                                "tree" : Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Go']), Tree('CC', ['and']), Tree('VB', ['defuse']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['bomb'])]), Tree('PP-MNR', [Tree('IN', ['in']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['cellar'])])])]), Tree('.', ['.'])])
                                                                },
                    
                "Carry_NPNP_NP" : {"sent" : "Carry meals to the library and cellar from the kitchen.",
                                   "tree": Tree('S', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Carry']), Tree('NP-A', [Tree('NP', [Tree('NNS', ['meals'])]), Tree('PP', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['library']), Tree('CC', ['and']), Tree('NN', ['cellar'])])])]), Tree('PP-CLR', [Tree('IN', ['from']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['kitchen'])])])]), Tree('.', ['.'])]),
                                   }
                }
    
    def __init__(self):
        self.tests = [self.coordination]
        
    def coordination(self,matcher):
        easy = self.easyDict
        hard = self.hardDict
        for cs in hard:
            print "Sent: ",hard[cs]["sent"]
            print "Tree: ",hard[cs]["tree"]
            
        
    
class exampePPAttachment(object):
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
                
