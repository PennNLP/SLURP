'''
Created on Oct 10, 2013
parse and tree are synonymous
@author: tad
'''
from nltk import Tree
from matchingTests import exampePPAttachment, exampleCoordination
from _matchingExceptions import NoSChildVP, PosTooDeep, VerbFrameCountError,SlotTreeCountError, NoRightSibling, TreeProcError, NodeNotFound, SlotNotFilledError
import sys
import copy
DEBUG = False

def main():
    testList = [exampePPAttachment().tests,exampleCoordination().tests]
    #testList = [exampleCoordination().tests]
    matcher = ParseMatcher(0,2)
    for tests in testList:
        for test in tests:
            test(matcher)

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
        '''Recursive traversal that augments a tree with the depth of each node attached to each node
            And also appends a unique identifier to each node in case of multiple lemmas
        '''        
        try:
            tree.node
            if self.depthdelim in tree.node: return True#Tree has already been augmented
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
        if len(possplit) > 1 and possplit[0] == '' and possplit[1] != '':
            return possplit[1], depthsplit[-1]
        
        return possplit[0],depthsplit[-1]        
        
    def leftmost_pos(self,tree,pos,cursor):
        '''Recursively returns the node for the leftmost pos
            Cursor is the current rightmost path for this traversal.
            Returned tree must be to the right of cursor by at least one branch
        '''
        #If tree only has 1 branch       
        if len(tree) == 1:
            #If that branch 
            if type(tree[0]) == type(''):
                npos,ndep = self.node_pos(tree)
                #If this is the pos we are looking for         
                if npos in pos: 
                    if len(cursor) == 1 and cursor[0] == 0:
                        #already have this node, return None
                        return None                   
                    else: return tree
            #If only child is a long skinny branch:
            for i,branch in enumerate(tree):                
                if i > cursor[0]:
                    #Just passed cursor, don't continue keeping track
                    cursor = [-1]                
                if i >= cursor[0]:
                    #If this branch is to the right of cursor
                    if type(branch) == type(''):
                        #leaf catch but we want the Tree
                        return None
                    else:
                        if cursor[0] != -1:
                            if len(cursor) == 1: return None                                          
                            else: res = self.leftmost_pos(branch,pos,cursor[1:])
                            return res
                        else:
                            res = self.leftmost_pos(branch,pos,cursor)
                            return res    
        #If more than one branch, call recursively on each of them                        
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
        path = []
        for branch in leafpath:
            tree = tree.pop(branch)
            path.append((branch,tree))
        foundRSibling = False
        parentpath = []
        #r for reverse node order
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
                                  "PREP": ["IN","TO"],
                                  "in" : ["IN"],
                                  "to" : ["TO"],
                                  "as" : ["as"]
                                  },
                        'LEX' : ['there']
                        
                        }   
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
            sys.stderr.write('tried to print_path for a leaf string or no path')
        elif len(path) == 2:
            leaf = self.th.remove_ulid(tree[path[0]][path[1]])
            return leaf
        else: 
            return self.get_leaf(tree[path[0]],path[1:])   
        
    def get_parent(self,tree,path):
        if len(path) < 2:
            sys.stderr.write('tried to print_path for a leaf string or no path')
        elif len(path) == 2:
            tree[path[0]][path[1]] = self.th.remove_ulid(tree[path[0]][path[1]])
            return tree[path[0]]
        else: 
            return self.get_parent(tree[path[0]],path[1:])   
         

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
            #last item of sLeftBranch is the branch to the VP, subtract one and you get the nearest left branch            
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
    
    def pp_acceptpath(self,prevpath,nextslot,next,path,tree):
        '''Returns True if the path points to a leaf that is an acceptable distance from the most
            recently filled slot's leaf, given the current slot that needs to be filled. "Acceptable distance"
            changes depending on the preposition (if it is one). 
            
            Carry the meals from the kitchen to the cellar. -> should allow "to [the cellar]" to fill the PREP slot next to "the kitchen"
            Carry the meals from the kitchen in the cellar to the hallway. -> should increment the cursor for "in [the cellar]" but keep looking for "to [the hallway]" if it exists whether or not [to] is embedded
            Carry the meals from the kitchen in the cellar. -> should fail on [[NP],[VB],[PP],[NP-Source],[PP],[NP-dest]] iff "in the cellar" is an embedded PP in "the kitchen in the cellar"
        
            Simply: Any embedded non-"to" preposition will return False, everything else will return True
             
            @input cur is the currently filled frame
            @input next is the slot that needs to be filled
            @input path points to the leaf that could fill the next slot
            @input tree that path belongs to            
        '''
        if nextslot[0] != "PREP": return True
        if nextslot[1] == 'to towards': return True
        if len(path) == 2 or len(prevpath) == 2: return False#siblings
        path = [next] + [w for w in path]
        prevparent = prevpath[:-3]#leafs are last two digits of path + phrase
        curhead = path[:-2]
        for i,n in enumerate(prevparent):
            if len(curhead) < i:
                #The previous path is a subset of the head of path => embedded
                return False
            elif curhead[i] != n:
                #Head of previous is not a subset of head of path => !embedded
                return True
        #The heads are (equal) => sisters
        return False
        
    def sequential_match_slots(self,next,subframe,base,tree):
        '''Match the slots in the subframe given the tree and base
            If the slot is for a preposition that accepts embedding, fill slots as if it were not embedded.
            Otherwise, increment the cursor as if it were the head but keep looking.
            
            @input base where are we right now  
        '''
        #next is the index of the branch of tree for the nearest right neighbor of v
        res = []
        tree = copy.deepcopy(tree)
        cursor = [-1]
        subpath = ""
        for slot in subframe:            
            if next < len(tree):     
                if next > cursor[0]: cursor = [-1]
                if len(cursor) > 1:
                    #First item of cursor is "next", pass cursor[1:]
                    path = self.get_path(tree[next],slot,cursor[1:])
                else:
                    path = self.get_path(tree[next],slot,cursor)
                #If a path to the head of the slot was not found
                while not path and next+1 < len(tree):
                    next += 1
                    #If next is ever greater than cursor[0], we are on a new branch and no longer care
                    if next > cursor[0]: cursor = [-1]
                    if len(cursor) > 1:
                        path = self.get_path(tree[next],slot,cursor[1:])
                    else:
                        path = self.get_path(tree[next],slot,cursor)                    
                if path:         
                    prevpath = subpath
                    subpath = [next] + [w for w in path]#subpath from tree, which is prototypically the VP 
                    full = base + subpath               #full path from root                          
                    cursor = subpath                    #Here, we increment the cursor but only fill the slot if it is acceptable
                    if len(res) == 0:
                        res.append((slot,full))       
                    elif self.pp_acceptpath(prevpath,slot,next,path,tree):   
                        res.append((slot,full))  
                    else:
                        pass                      
                else:
                    res.append((slot,SlotNotFilledError))
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
    
    def frames_match_frame (self, svo, tree):
        '''Format svo into the format frames expects:
            {'<POS>' : tree,...}        '''
        s, v, o = svo
        res = {}
        for i in s:
            res[i[0][1]] = self.get_parent(tree,i[1])
        res['VERB'] = self.get_parent(tree,v)
        for i in o:
            res[i[0][1]] = self.get_parent(tree,i[1])
        return res    
    
    def invalid_pos(self,frame):
        for slot in frame:
            if not slot[0] in self.pos_map:
                sys.stderr.write("Slot part of speech unknown for slot:" + str(slot))
                return True
        return False
            
    def check_for_augment(self,tree):
        npos, ndepth = self.th.node_pos(tree)
        try:
            int(ndepth[-1])
        except ValueError:
            sys.stderr.write("Tree does not seem to be augmented for depth, please run depth_ulid_augment ONCE and then try calling this method again.")
        
    def match_frame(self,frame,parse):
        '''Try to match the frame to the parse'''
        try:
            if self.invalid_pos(frame):
                return None            
            self.check_for_augment(parse)
            verbslots = [(i,w) for i,w in enumerate(frame) if w[0] == "VERB"]            
            if len(verbslots) > 1: raise VerbFrameCountError
            verbindex, verbslot = verbslots[0]
            verbpath = self.get_path(parse,verbslot)
            pmatch = self.proximity_match_frame(verbpath,verbindex,frame,parse)
            s,v,o = pmatch
            if True in [(type(w[1])==type) for w in s]: 
                sys.stderr.write("Error finding subject for frame. "+ str(s)+"\n")
                return None
            elif True in [(type(w[1])==type) for w in o]: 
                sys.stderr.write("Error finding object for frame. "+ str(o)+"\n")
                return None
            if DEBUG: self.print_svo(pmatch[0],pmatch[1],pmatch[2],parse)
            fmatch = self.frames_match_frame(pmatch,parse)             
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
        return fmatch
        

if __name__=="__main__":
    main()
            
            
                
        
        
        