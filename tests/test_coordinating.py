'''
Created on Oct 17, 2013

@author: taylor
'''
from nltk import Tree
import sys
from semantics.coordinating import Split, Condition
from semantics.matching import ParseMatcher

def main():
    print 'hello world'
    matcher = ParseMatcher(0,2)
    testList = [exampleCoordination().tests]
    for category in testList:
        for test in category:
            test(matcher)
    
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
    ccConditionalDict = {"CC and adv conditional" : {"sent" : "Go to the cellar and if you see a bomb, activate your camera.",
                                          "tree" : Tree('S', [Tree('S-A', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Go']), Tree('PP-CLR', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['cellar'])])])])]), Tree('CC', ['and']), Tree('S-A', [Tree('SBAR-ADV', [Tree('IN', ['if']), Tree('S-A', [Tree('NP-SBJ-A', [Tree('PRP', ['you'])]), Tree('VP', [Tree('VBP', ['see']), Tree('NP-A', [Tree('DT', ['a']), Tree('NN', ['bomb'])])])])]), Tree(',', [',']), Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['activate']), Tree('NP-A', [Tree('PRP$', ['your']), Tree('NN', ['camera'])])])]), Tree('.', ['.'])]),
                                          "frames" : []                                      
                                      },
              "CC and tmp conditional" : {"sent" : "Go to the cellar and when you see a bomb, activate your camera.",
                                          "tree" : Tree('S', [Tree('S-A', [Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['Go']), Tree('PP-CLR', [Tree('TO', ['to']), Tree('NP-A', [Tree('DT', ['the']), Tree('NN', ['cellar'])])])])]), Tree('CC', ['and']), Tree('S-A', [Tree('SBAR-TMP', [Tree('WHADVP-0', [Tree('WRB', ['when'])]), Tree('S-A', [Tree('NP-SBJ-A', [Tree('PRP', ['you'])]), Tree('VP', [Tree('VBP', ['see']), Tree('NP-A', [Tree('DT', ['a']), Tree('NN', ['bomb'])]), Tree('ADVP-0', [Tree('-NONE-', ['*T*'])])])])]), Tree(',', [',']), Tree('NP-SBJ-A', [Tree('-NONE-', ['*'])]), Tree('VP', [Tree('VB', ['activate']), Tree('NP-A', [Tree('PRP$', ['your']), Tree('NN', ['camera'])])])]), Tree('.', ['.'])]),
                                          "frames": [],
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
        #self.tests = [self.coordination]
        self.tests = [self.conditional]
        
    def coordination(self,matcher):
        easy = self.easyDict
        hard = self.hardDict
        for cs in hard:
            print "Sent: ",hard[cs]["sent"]
            print "Tree: ",hard[cs]["tree"]
            
    def conditional(self,matcher):
        dict = self.ccConditionalDict
        splitter = Split()
        conditioner = Condition()
        for sent in dict:
            tree = dict[sent]["tree"]
            sent = dict[sent]["sent"]            
            print "Sent: ",sent
            print "Tree: ",tree
            matcher.th.depth_ulid_augment(tree,0)
            trees = splitter.split_on_cc(tree)            
            for t in trees:
                conditions = conditioner.split_on_sbar(t)                
                #And match when get frames
            
if __name__=="__main__":
    main()
                
