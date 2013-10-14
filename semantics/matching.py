'''
Created on Oct 10, 2013
parse and tree are synonymous
@author: tad
'''
from nltk import Tree
from _matchingExceptions import NoSChildVP, PosTooDeep, VerbFrameCountError, SlotTreeCountError
import sys
import copy
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
        matcher.th.depth_augment(self.correct,0)
        matcher.th.depth_augment(self.incorrect,0)
        for frame in self.frame_list:
            #cmatch = matcher.match_frame(frame,\
            #                     self.correct)
            icmatch = matcher.match_frame(frame,\
                                 self.incorrect)


class TreeHandler(object):
    depthdelim = "."
    posdelim = "-"    

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
        
    def leftmost_pos(self,tree,pos):
        '''Recursively returns the node for the leftmost pos'''
        
        if len(tree) == 1:
            #If leaf is str
            if type(tree[0][0]) == type(''):                
                depthsplit = tree.node.split(self.depthdelim)#Split on depth delim
                possplit = depthsplit[0].split(self.posdelim)#split on pos delim
                #If this is the pos we are looking for
                if possplit[0] == '' and possplit[1] != '':
                    if possplit[1] in pos:
                        return tree                   
                elif possplit[0] in pos:                    
                    return tree
            for branch in tree:
                if type(branch) == type(''):
                    return None
                else:
                    res = self.leftmost_pos(branch,pos)
                    return res
        for branch in tree:
            res = self.leftmost_pos(branch,pos)
            if res:
                return res
        return None
        
    def get_main_pos_path(self,tree,pos,maxPosDepth):
        '''Return path to the shallowest leftmost VB'''        
        node = self.leftmost_pos(tree, pos)
        path = self.get_path_to_node(tree,node)
        if maxPosDepth != -1 and len(path) > maxPosDepth:            
            raise PosTooDeep(pos)        
        return path    
        
    def get_path_to_node(self,tree,node):
        leaves = tree.leaves()
        #accomodate nullsubjects here
        node is the phrase, which is good except we are trying to match on leafs for heads, not phrases        
        index = leaves.index(node[0])
        return tree.leaf_treeposition(index)
    
    def nearest_right_sibling(self,leafpath,tree):
        '''Returns the path(inclusive) to the branch of the nearest right sibling of leafpath
            (1,0,1,1,0,0) returns [1,0,1,1] iff the subtree at [1,0,1,1] has > 1 children
            destructive mthd, makes a copy of tree            
        '''
        #destructive mthd, make copy
        tree = copy.deepcopy(tree)
        #b for branch, r for reverse node order        
        path = []
        for branch in leafpath:
            tree = tree.pop(branch)
            path.append((branch,tree))
        foundSibling = False
        res = []
        for branch,r in reversed(path):
            if foundSibling:
                res.insert(0,branch)
            elif len(r) > 1:
                foundSibling = True
        return res
                
    def nearest_left_sibling(self,leafpath):
        '''Returns the path(inclusive) to the branch of the nearest left sibling of leafpath
            (1,0,1,1,0,0) returns [1,0,1,1]
            (1,0,0) returns [1]
        '''
        res = []
        foundLeft = False
        for b in reversed(leafpath):
            if foundLeft:
                res.insert(0,b)
            elif b > 0:
                foundLeft = True 
                res.insert(0,b)   
        return res
    
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
#         self.maxVBDepth = smatch + 3
#         self.vbDepth = -1
#         self.subjectDepth = -1
#         self.directObjectDepth = -1
#         self.indirectObjectDepth = -1
        self.role_dict= {
                         'subject' : 1,
                         'verb' : 2,
                         'prep' : 3,
                         'object' : 4
                         }
        self.pos_map = { 'VERB' : ['VB'],
                        'NP' : ['NONE','NNS','NN','NNP','NNPS'],
                        'PREP' : {"to towards" : ["TO"],
                                  "PREP": ["IN"]}}   
        self.depth_map = {'maxVB' : smatch + 3,
                          'maxNONE' : -1,
                          'maxTO': -1,
                          'maxIN': -1,
                          'maxPREP': -1}    
        
    def get_path(self,tree,slot):
        pos, role, secondary, tertiary = slot      
        heads = self.pos_map[pos]
        if type(heads) == type({}):
            heads = heads[role]
        maxd = -1
        for head in heads:            
            if 'max'+head in self.depth_map:
                maxd = self.depth_map['max'+head]                    
        
        mainpos = self.th.get_main_pos_path(tree,head,maxd)
        print 'path to mainpos for slot(',slot,') ',mainpos
        return mainpos 
        
    def match_subject(self,subframe,v,tree):
        '''    Match the subject given the subframe([]), path(tup) and syntaxparse(Tree)
                return [path] to each slot       
            subject must be to the left of v from root 
        ''' 
        res = []
        #rev = v.reverse()
        nearestLeftBranch = self.th.nearest_left_sibling(v)       
        for slot in subframe:            
            res.append(self.get_path(tree,slot))
        return res
    
    def match_object(self,subframe,v,tree):
        '''    Match the object given the subframe([]), path(tup) and syntaxparse(Tree)
                return [path] to each slot        
            object must be to the right of v
        ''' 
        tree = copy.deepcopy(tree)
        res = []       
        nearestRightBranch = self.th.nearest_right_sibling(v,tree)
        #nearest sibling methods return inclusive so we can pop greater than the last elmt
        for i in nearestRightBranch[:-1]:
            tree = tree.pop(i)
        #At this point, tree should be the constituent VP of the main verb
        next = nearestRightBranch[-1]+1
        for slot in subframe:
            if next < len(tree):
                res.append(self.get_path(tree.pop(next),slot))
            else:
                raise SlotTreeCountError            
        return res      
        
    def proximity_match_frame(self,v,center,frame,tree):
        '''Given a v(tup) and center(int) match the frame around that path.'''        
        left = frame[:center]#S
        #V
        right = frame[center+1:]#O
        s = self.match_subject(left,v,tree)
        o = self.match_object(right,v,tree)
        return [s,v,o]
        
    def match_frame(self,frame,parse):
        '''Try to match the frame to the parse'''
        try:            
            verbslots = [(i,w) for i,w in enumerate(frame) if w[0] == "VERB"]            
            if len(verbslots) > 1: raise VerbFrameCountError
            verbindex, verbslot = verbslots[0]
            verbpath = self.get_path(parse,verbslot)
            match = self.proximity_match_frame(verbpath,verbindex,frame,parse)      
            print 'Done matching slots in frame(',frame,')'              
        except PosTooDeep, pos:
            sys.stderr.write(pos+" found but too deep given max part-of-speech depth\n")
        except VerbFrameCountError:
            sys.stderr.write(str(frame)+" contains too many or too few verbs...error\n")
        except SlotTreeCountError:
            #Not all the slots could be filled for this frame given this parse
            return None
        except AttributeError, e:
            print 'Attribute error trying to match frame: ',str(e)
        return match
        

if __name__=="__main__":
    main()
            
            
                
        
        