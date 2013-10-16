'''
Created on Oct 10, 2013
parse and tree are synonymous
@author: tad
'''
from nltk import Tree
from _matchingExceptions import NoSChildVP, PosTooDeep, VerbFrameCountError,SlotTreeCountError, NoRightSibling, TreeProcError, NodeNotFound, SlotNotFilledError
import sys
import copy
DEBUG = False
def main():
    testlist = [exampePPAttachment().tests]
    for tests in testlist:
        for test in tests:
            test()
    
class exampePPAttachment(object):
    '''    Syntax examples for the two sentences:
            Carry the meals from the kitchen to the rooms.
            *Carry the meals from the kitchen to the cafeteria. -> yield an incorrect syntax parse
        Used to develop strict verbframe matching and pp attachment.
    '''
    correct = Tree('''(S\n  (NP-SBJ-A (-NONE- *))\n  (VP\n    (VB Carry)\n    (NP-A (DT the) (NNS meals))\n    (PP-CLR (IN from) (NP-A (DT the) (NN kitchen)))\n    (PP-CLR (TO to) (NP-A (DT the) (NNS rooms))))\n  (. .))''')
    incorrect = Tree('''(S\n  (NP-SBJ-A (-NONE- *))\n  (VP\n    (VB Carry)\n    (NP-A (DT the) (NNS meals))\n    (PP-CLR\n      (IN from)\n      (NP-A\n        (NP (DT the) (NN kitchen))\n        (PP (TO to) (NP-A (DT the) (NN cafeteria))))))\n  (. .))''')
    
    pp_attach_framelist = [
                  [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', '')],
                  [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],
                  [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],    
                  [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', ''), ('PREP', 'PREP', '', ''), ('NP', 'Source', '', '')],   
                  [('NP', 'Theme', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Value', '', '')],    
                  [('NP', 'Theme', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Beneficiary', '', ''), ('NP', 'Value', '', '')]                
                ]
    crazyframes = {"SUBPHRASE_PASS": 
                   [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('DT','DT','',''),('NP', 'Theme', '', ''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')],
                   "SUBPHRASE_FAIL": 
                   [('NP', 'Agent', '', ''), ('VERB', 'VERB', '', ''), ('NP', 'Theme', '', ''),('DT','DT','',''), ('PREP', 'to towards', '', ''), ('NP', 'Destination', '', '')]

                   }
     
    def __init__(self):
        #tests = [self.cursor_test,self.pp_test]
        tests = [self.pp_test]
        self.tests = tests
    
    def pp_test(self):
        matcher = ParseMatcher(0,2)
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
            s,v,o = match
            if True in [(type(w)==type) for w in s]: sys.stderr.write("Error finding subject for frame. "+ str(s)+"\n"); sys.stderr.flush()
            elif True in [(type(w)==type) for w in o]: sys.stderr.write("Error finding object for frame. "+ str(o)+"\n"); sys.stderr.flush()
            else:            
                matcher.print_svo(s,v,o, tree1)
            print "Incorrect tree frame match:"
            match = matcher.match_frame(frame, tree2)
            s,v,o = match
            if True in [(type(w)==type) for w in s]: sys.stderr.write("Error finding subject for frame. "+ str(s)+"\n")
            elif True in [(type(w)==type) for w in o]: sys.stderr.write("Error finding object for frame. "+ str(o)+"\n")
            else:
                matcher.print_svo(s,v,o, tree2)
            
    def cursor_test(self):
        matcher = ParseMatcher(0,2)
        matcher.th.depth_ulid_augment(self.correct,0)
        tree = self.correct
        print "Tests for tree: "
        print tree
        for key in self.crazyframes:
            print "Running test(",key,")"
            frame = self.crazyframes[key]            
            match = matcher.match_frame(frame,tree)
            matcher.print_svo(match[0],match[1],match[2], tree)


class TreeHandler(object):
    depthdelim = "."
    posdelim = "-"
    
    def __init__(self):
        #A unique leaf identifier in case there are identical leaves
        self.ulid = 0
        
    def get_ulid(self):
        self.ulid += 1
        return self.ulid-1
    
    def remove_ulid(self,leaf):
        return leaf.split('__')[0]

    def common_acestor(self,tree,indexa,indexb):
        '''Returns the path to the common ancestor of the two indices'''
        patha = tree.leaf_treeposition(indexa)
        pathb = tree.leaf_treeposition(indexb)
        if len(patha) <= len(pathb):
            return [w for i,w in enumerate(patha) if w == pathb[i]]
        return [w for i,w in enumerate(pathb) if w == patha[i]]

    def depth_ulid_augment(self,tree,depth):
        '''Recursive traversal that augments a tree with the depth of each node attached to each node'''        
        try:
            tree.node
        except AttributeError:
            if DEBUG: print tree
        else:
            #augment tree
            tree.node+=self.depthdelim+str(depth)
            if DEBUG: print '(',tree.node,
            for i,child in enumerate(tree):
                if type(child) == str:
                    tree[i] += '__'+str(self.get_ulid())+'__' 
                self.depth_ulid_augment(child,depth+1)
            if DEBUG: print ')',    
            
    def get_subtree(self,tree,path):
        '''Recursive traversal that returns the subtree when no more path'''
        if len(path) == 0:
            return tree
        self.get_subtree(tree[path[0]],path[1:])
        
    def node_pos(self,tree):
        depthsplit = tree.node.split(self.depthdelim)#Split on depth delim
        possplit = depthsplit[0].split(self.posdelim)#split on pos delim
        if possplit[0] == '' and possplit[1] != '':
            return possplit[1], depthsplit[-1]
        return possplit[0],depthsplit[-1]        
    
#     def leaf_parent(self,tree):
#         '''Dig down and get the parent of what should be the only leaf of this tree'''
#         if tree[0]
        
    def leftmost_pos(self,tree,pos,cursor):
        '''Recursively returns the node for the leftmost pos
            Cursor is the current rightmost path for this traversal.
        '''
        #If tree only has 1 branch       
        if len(tree) == 1:
            #If that branch 
            #if type(tree[0][0]) == type(''):                
            if type(tree[0]) == type(''):
                npos,ndep = self.node_pos(tree)
                #If this is the pos we are looking for         
                if npos in pos: 
                    if len(cursor) == 1 and cursor[0] == 0:
                        #already have this node, return None
                        return None                   
                    else: return tree
            for i,branch in enumerate(tree):
                if i > cursor[0]:
                    cursor = [-1]
                if i >= cursor[0]:
                    if type(branch) == type(''):
                        return None
                    else:
                        if cursor[0] != -1:
                            if len(cursor) == 1: return None                                          
                            else: res = self.leftmost_pos(branch,pos,cursor[1:])
                            return res
                        else:
                            res = self.leftmost_pos(branch,pos,cursor)
                            return res                            
        for i,branch in enumerate(tree):
            if i > cursor[0]:
                cursor = [-1]
            if i >= cursor[0]:
                if cursor[0] != -1:
                    if len(cursor) == 1: return None                                         
                    else: res = self.leftmost_pos(branch,pos,cursor[1:])
                    if res:
                        return res                
                else:
                    res = self.leftmost_pos(branch,pos,cursor)
                    if res:
                        return res                    
        return None
        
    def get_main_pos_path(self,tree,pos,maxPosDepth,cursor=[-1]):
        '''Return path to the shallowest leftmost VB'''      
        node = self.leftmost_pos(tree, pos,cursor)
        if not node:
            return False
        path = self.get_path_to_node(tree,node)
        if maxPosDepth != -1 and len(path) > maxPosDepth:            
            raise PosTooDeep(pos)        
        return path    
        
    def get_path_to_node(self,tree,node):
        leaves = tree.leaves()
        #accomodate nullsubjects here
        #node is the phrase, which is good except we are trying to match on leaves/heads, not phrases        
        index = leaves.index(node[0])
        return tree.leaf_treeposition(index)
    
    def nearest_right_sibling(self,leafpath,tree):
        '''Returns the path(inclusive of the first branch in leafpath past the parent)
            to the parent of the nearest right sibling of leafpath
            (1,0,1,1,0,0) returns [1,0,1,1,0] iff the parent(subtree) at [1,0,1,1] has > 1 children
            destructive mthd, makes a copy of tree            
        '''
        #destructive mthd, make copy
        tree = copy.deepcopy(tree)
        #b for branch, r for reverse node order        
        path = []
        for branch in leafpath:
            tree = tree.pop(branch)
            path.append((branch,tree))
        foundRSibling = False
        parentpath = []
        #Reverse order traversal along path
        last = -1
        for branch,r in reversed(path):
            #Because the list is popped trees, the first branch with any length
            if not foundRSibling and type(r) != str and len(r) > 0:
                foundRSibling = True
            elif not foundRSibling and branch > 0:
                raise NoRightSibling 
            if foundRSibling:
                #initialize 
                if len(parentpath) == 0:
                    parentpath.insert(0,last[0])
                parentpath.insert(0,branch)     
            last = (branch,r)        
        else:            
            return parentpath
        raise TreeProcError("Error finding nearest right sibling of leafpath: "+str(leafpath))
                
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
        self.pos_map = {'DT' : ['DT'], 
                        'VERB' : ['VB'],
                        'NP' : ['NONE','NNS','NN','NNP','NNPS'],
                        'PREP' : {"to towards" : ["TO"],
                                  "PREP": ["IN"]}}   
        self.depth_map = {'maxVB' : smatch + 3,
                          'maxNONE' : -1,
                          'maxTO': -1,
                          'maxIN': -1,
                          'maxPREP': -1}    
        
    def get_path(self,tree,slot,cursor=[-1]):
        '''Return the path to the head of the slot'''        
        pos, role, secondary, tertiary = slot      
        heads = self.pos_map[pos]
        if type(heads) == type({}):
            heads = heads[role]
        maxd = -1
        for head in heads:            
            if 'max'+head in self.depth_map:
                maxd = self.depth_map['max'+head]        
        mainpos = self.th.get_main_pos_path(tree,heads,maxd,cursor)
        if DEBUG : print 'path to mainpos for slot(',slot,') ',mainpos
        return mainpos 
        
    def pop_path(self,tree,path):
        if len(path) < 2:
            sys.stderr.write('tried to pop_path for a path with no length')
        elif len(path) == 2:
            tree.pop(path[0])
        else: 
            self.pop_path(tree[path[0]],path[1:])
            
    def get_leaf(self,tree,path):    
        if len(path) < 2:
            sys.stderr.write('tried to print_path for a path with no length')
        elif len(path) == 2:
            leaf = self.th.remove_ulid(tree[path[0]][path[1]])
            return leaf
        else: 
            return self.get_leaf(tree[path[0]],path[1:])
    

    def match_subject_object(self,left,right,v,tree):
        '''    Match the object/subject given the subframes(left,right), v(path to VB) and tree(full tree)
                return [path] to each slot        
            @input v is the path to the VP + the last item is the VB branch
            @input tree is the VP itself
            @input subframe is the object subframe 
            object must be to the right of v
        '''   
        stree = copy.deepcopy(tree)
        otree = copy.deepcopy(tree)        

        '''Nearest siblings
            nearest sibling methods return inclusive of head so we can pop greater than the last elmt
        '''
        sLeftBranch = self.th.nearest_left_sibling(v)   
        for i in sLeftBranch[:-1]:
            stree = stree.pop(i)
        else:
            #last item of sLeftBranch is the branch to the verb, subtract one and you get the nearest left branch            
            sLeftBranch[-1] -= 1
            stree = stree.pop(sLeftBranch[-1])
            

        oRightBranch = self.th.nearest_right_sibling(v,otree)        
        for i in oRightBranch[:-1]:
            otree = otree.pop(i)
            
        '''At this point,   otree should be the constituent VP of the main verb
                  and    stree should be the * branch to the left of the VP'''        
        snext = 0
        onext = oRightBranch[-1] + 1
        
        s = self.sequential_match_slots(snext,left, sLeftBranch, stree)
        o = self.sequential_match_slots(onext,right, oRightBranch[:-1], otree)
        return s,o
        
    def sequential_match_slots(self,next,subframe,base,tree):
        '''Match the slots in the subframe given the tree and base        
            @input base where are we right now  
        '''
        #next is the index of the branch of tree for the nearest right neighbor of v
        res = []
        tree = copy.deepcopy(tree)
        cursor = [-1]
        for slot in subframe:            
            if next < len(tree):     
                if next > cursor[0]: cursor = [-1]
                if len(cursor) > 1:
                    path = self.get_path(tree[next],slot,cursor[1:])
                else:
                    path = self.get_path(tree[next],slot,cursor)
                #If a path to the head of the slot was not found
                while not path and next+1 < len(tree):
                    next += 1
                    if next > cursor[0]: cursor = [-1]
                    if len(cursor) > 1:
                        path = self.get_path(tree[next],slot,cursor[1:])
                    else:
                        path = self.get_path(tree[next],slot,cursor)
                    
                if path:         
                    #subpath from tree, which is prototypically the VP
                    subpath = [next] + [w for w in path] 
                    #full path from root of v, which is the path to the VP
                    full = base + subpath
                    cursor = subpath
                    #self.pop_path(tree,subpath)
                                        
                    res.append((slot,full))
                    #res.append(full)
                else:
                    res.append(SlotNotFilledError)
            else:
                raise SlotTreeCountError            
        return res        
    
    def match_object(self,subframe,v,tree):
        '''    Match the object given the subframe([]), path(tup) and syntaxparse(tree)
                return [path] to each slot        
            @input v is the path to the VP + the last item is the VB branch
            @input tree is the VP itself
            @input subframe is the object subframe 
            object must be to the right of v
        '''        
        tree = copy.deepcopy(tree)
        res = []       
        nearestRightBranch = self.th.nearest_right_sibling(v,tree)
        #nearest sibling methods return inclusive so we can pop greater than the last elmt
        for i in nearestRightBranch[:-1]:
            tree = tree.pop(i)
        #At this point, tree should be the constituent VP of the main verb
        next = nearestRightBranch[-1] + 1
        #next is the index of the branch of tree for the nearest right neighbor of v
        for slot in subframe:            
            if next < len(tree):     
                path = self.get_path(tree[next],slot)
                #If a path to the head of the slot was not found
                while not path and next+1 < len(tree):
                    next += 1
                    path = self.get_path(tree[next],slot)
                if path:         
                    #subpath from tree, which is prototypically the VP                    
                    subpath = [next] + [w for w in path] 
                    #full path from root of v, which is the path to the VP
                    full = nearestRightBranch[:-1] + subpath
                    self.pop_path(tree,subpath)
                    res.append(full)
                else:
                    return SlotNotFilledError
            else:
                raise SlotTreeCountError            
        return res      
        
    def print_svo(self,s,v,o,tree):
        print 'subject: ',[(w[0],w[1],self.get_leaf(tree,w[1])) for w in s]
        #print 'subject: ',[(w,self.get_leaf(tree,w)) for w in s]
        print 'verb: ',self.get_leaf(tree,v)
        #print 'object: ',[(b,self.get_leaf(tree,b)) for b in o]
        print 'object: ',[(b[0],b[1],self.get_leaf(tree,b[1])) for b in o]
        
            
    def proximity_match_frame(self,v,center,frame,tree):
        '''Given a v(tup) and center(int) match the frame around that path.'''        
        left = frame[:center]#S
        #V
        right = frame[center+1:]#O
        s,o = self.match_subject_object(left,right,v,tree)
        if DEBUG: self.print_svo(s,v,o,tree)
        return [s,v,o]
        
    def match_frame(self,frame,parse):
        '''Try to match the frame to the parse'''
        try:            
            verbslots = [(i,w) for i,w in enumerate(frame) if w[0] == "VERB"]            
            if len(verbslots) > 1: raise VerbFrameCountError
            verbindex, verbslot = verbslots[0]
            verbpath = self.get_path(parse,verbslot)
            match = self.proximity_match_frame(verbpath,verbindex,frame,parse)             
        except PosTooDeep, pos:
            sys.stderr.write(pos+" found but too deep given max part-of-speech depth\n")
        except VerbFrameCountError:
            sys.stderr.write(str(frame)+" contains too many or too few verbs...error\n")
        except SlotTreeCountError:
            #Not all the slots could be filled for this frame given this parse
            return None
        except AttributeError, e:
            print 'Attribute error trying to match frame: ',str(e)
        except:
            raise
        return match
        

if __name__=="__main__":
    main()
            
            
                
        
        
        