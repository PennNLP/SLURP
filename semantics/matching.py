'''
Created on Oct 10, 2013
parse and tree are synonymous
@author: tad
'''
from nltk import Tree
from _matchingExceptions import NoSChildVP
DEBUG = False
def main():
    tests = [exampePPAttachment()]
    for test in tests:
        test.run_test()
    
class exampePPAttachment(object):
    '''    Syntax examples for the two sentences:
            Carry the meals from the kitchen to the rooms.
            *Carry the meals from the kitchen to the cafeteria. -> yield an incorrect syntax parse
        Used to develop strict verbframe matching and pp attachment.
    '''
    correct = Tree('''(S\n  (NP-SBJ-A (-NONE- *))\n  (VP\n    (VB Carry)\n    (NP-A (DT the) (NNS meals))\n    (PP-CLR (IN from) (NP-A (DT the) (NN kitchen)))\n    (PP-CLR (TO to) (NP-A (DT the) (NNS rooms))))\n  (. .))''')
    incorrect = Tree('''(S\n  (NP-SBJ-A (-NONE- *))\n  (VP\n    (VB Carry)\n    (NP-A (DT the) (NNS meals))\n    (PP-CLR\n      (IN from)\n      (NP-A\n        (NP (DT the) (NN kitchen))\n        (PP (TO to) (NP-A (DT the) (NN cafeteria))))))\n  (. .))''')
    frame_list = [
                  [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', '')],
                  [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],
                  [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],    
                  [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', '')],   
                  [('NP', 'Theme', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Value', '', '')],    
                  [('NP', 'Theme', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Beneficiary', '', ''), ('NP', 'Value', '', '')]                
                ]   
     
    def run_test(self):
        matcher = ParseMatcher(0,2)
        cmatch = matcher.match_frame(exampePPAttachment.frame_list,\
                                 exampePPAttachment.correct)
        icmatch = matcher.match_frame(exampePPAttachment.frame_list,\
                                 exampePPAttachment.incorrect)


class TreeHandler(object):
    depthdelim = "."
    def common_acestor(self,tree,indexa,indexb):
        '''Returns the path to the common ancestor of the two indices'''
        patha = tree.leaf_treeposition(indexa)
        pathb = tree.leaf_treeposition(indexb)
        if len(patha) <= len(pathb):
            return [w for i,w in enumerate(patha) if w == pathb[i]]
        return [w for i,w in enumerate(pathb) if w == patha[i]]

    def depth_augment(self,tree,depth):
        '''Recursive traversal that augments a tree with the depth of each node attached to each node'''        
        try:
            tree.node
        except AttributeError:
            if DEBUG: print tree
        else:
            #augment tree
            tree.node+=self.depthdelim+str(depth)
            if DEBUG: print '(',tree.node,
            for child in tree:
                self.depth_augment(child,depth+1)
            if DEBUG: print ')',    
            
    def get_subtree(self,tree,path):
        '''Recursive traversal that returns the subtree when no more path'''
        if len(path) == 0:
            return tree
        self.get_subtree(tree[path[0]],path[1:])
        
    def leftmost_pos(self,tree,pos,maxVBdepth=2):
        if len(tree) == 1:
            if type(tree[0][0]) == type(''):
                #If leaf is str
                nodesplit =tree.node.split(self.depthdelim) 
                if nodesplit[0] == pos:
                    if int(nodesplit[-1]) <= maxVBdepth: 
                        return tree
                return None#dead leaf 
        for branch in tree:
            res = self.leftmost_pos(branch,pos)
            if res:
                return res
            

                    
        
    def get_main_verb_path(self,tree):
        '''Return path to the shallowest leftmost VB'''
        for i,branch in enumerate(tree):
            if branch.node == self.s_child_vp:
                return i            
        raise NoSChildVP
    
     
        

class ParseMatcher(object):
    '''
    This class keeps track of state (traversal) information
    given a verbnet frame and syntax parse.
    strictMatching is a numeric value that sets the acceptable depth distance from the main VP
    strictPPMatching is a numeric value that sets the acceptable depth distance from the NP head
    '''
    
    def __init__(self,smatch,sppmatch):
        '''
        Constructor
        '''
        self.strictMatching = smatch
        self.strictPPMatching = sppmatch
        self.th = TreeHandler()
        self.minVbDepth = smatch + 2
        self.vbDepth = -1
        self.subjectDepth = -1
        self.directObjectDepth = -1
        self.indirectObjectDepth = -1
        self.role_dict= {
                         'subject' : 1,
                         'verb' : 2,
                         'prep' : 3,
                         'object' : 4
                         }       
        
    def match_frame(self,frame,parse):
        '''Try to match the frame to the parse'''
        print 'match_frame Tree before: ',parse
        self.th.depth_augment(parse,0)
        print 'match_frame Tree after: ',parse
        mainverb = self.th.leftmost_pos(parse, "VB")        
        print 'mainVP: ',mainverb

if __name__=="__main__":
    main()
            
            
                
        
        