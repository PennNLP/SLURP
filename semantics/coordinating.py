'''
Created on Oct 18, 2013

@author: taylor
'''
from treehandler import TreeHandler
import sys
from _matchingExceptions import NoRightSibling, NoLeftSibling, UnlevelCCSiblings
import copy
DEBUG = False
        
class Condition(object):
    '''Class to extract S trees from a parent tree that contains conditional children.'''
    accepted_conditions = ["SBAR-TMP","SBAR-ADV"]
    accepted_punc = [","]
    def __init__(self):
        self.th = TreeHandler()
        
    def split_on_sbar(self,tree):
        '''Return the subtrees of the tree if any SBAR present'''
        cursor = [-1]
        for condition in self.accepted_conditions:
            path = self.th.leftmost_pos(tree, condition, cursor)
            if path:
                print path                
        
class Split(object):
    '''Class to split a syntax tree on CCs by replacing sibling nodes of the CC with the parent.'''
    validCC = "and"
    listCC = "," 
    def __init__(self):
        self.th = TreeHandler()     
        
    def sibling_cc_path(self,ccpath,tree=None):
        '''Returns the nearest appropriate sibling to ccpath given a tree (or not)
            caveat #1 finding the leftmost sibling does not require a tree, so we can easily overload this method
            caveat #2 pop_path requires the leaf so we append a meaningless -1
        '''
        if tree:
            path = self.th.nearest_right_sibling(ccpath,tree)
            path[-1] += 1
        else: 
            path = self.th.nearest_left_sibling(ccpath)
            path[-1] -= 1#actual left branch
        path.append(-1)#Faking the leaf
        return path
           
    def pop_comma_and(self,tree):
        '''Given a tree, pop the comma of any siblings ", and" as in "the horse, donkey, and carriage."
            Functionally they are redundant.        
        '''
        prev = (0,'')
        while True:
            leaves = tree.leaves()
            for i, leaf in enumerate(leaves):
                dleaf = self.th.remove_ulid(leaf)
                if self.th.remove_ulid(prev[1]) == "," and dleaf == "and":
                    path = tree.leaf_treeposition(prev[0])
                    break
                prev = (i,leaf)
            else:
                return
            self.th.pop_path(tree,path)        
        
        
    def pos_split(self,tree,pos,possibleParents=["S","VP","NP"],desiredCC="and",ccpos="CC"):
        '''The strategy for this is to find the CC in the VP (like the VB in matching),
            find the nearest left or right siblings and if they are heads, pop   
            
            @input tree input tree to split on
            @input pos is the part of speech we are splitting on
            @input possibleParents are the possible phrase parents currently supported
            @input desiredCC is the lemma of the CC we are looking for
            @input ccpos is the cc part of speech we are looking for
        '''
        res = []        
        cursor = [-1]
        cccount = self.num_ccs(tree,desiredCC) 
        for i in range(cccount):
            ccpath = list(self.th.get_main_pos_path(tree, ccpos, -1, cursor=cursor))
            parentPhrase = self.th.which_parent(tree, ccpath, possibleParents, -1)
            parentpos = parentPhrase.split(self.th.depthdelim)[0].split(self.th.posdelim)[0]
            if parentpos == pos:                               
                left = self.sibling_cc_path(ccpath) 
                right = self.sibling_cc_path(ccpath,tree=tree);
                if len(left) != len(ccpath) != len(right):
                    raise UnlevelCCSiblings
                lefttree = copy.deepcopy(tree)
                #Instead of popping, need to replace parent with correct branch 
                temp = self.th.pop_path_cc(lefttree,right,ccpath)
                if temp:
                    #If splitting on S, temp will be the parent replacement
                    lefttree = temp
                #Recurse on "," list
                lefttrees = self.pos_split(lefttree,pos,possibleParents=possibleParents,desiredCC=self.listCC,ccpos=self.listCC)                
                res.extend(lefttrees) #Copy and put left branch in results and keep going
                self.th.pop_left(tree,ccpath)#Pop everything to the left of ccpath and keep looking
                #self.th.pop_path_cc(tree, left,ccpath)#Pop for real, keep looking
                cursor = [-1]
            else:
                if DEBUG: print pos,' CC: ',ccpath
                cursor = ccpath
        if len(tree) == 1:
            #Only one branch-> split on S, consume
            tree = tree[0]  
        res.append(tree)               
        return res
        
    def num_ccs(self,tree,cc):
        return [self.th.remove_ulid(w) in cc for w in tree.leaves()].count(True)

    def split_on_cc(self,tree):
        '''Split on "and"...extend to other CCs later.        
        '''
        self.pop_comma_and(tree)#Prepare the tree
        numCCs = self.num_ccs(tree,self.validCC)
        if numCCs < 1:
            return [tree]
        self.th.depth_ulid_augment(tree, 0)
        try:        
            ssplit = self.pos_split(tree,"S",desiredCC=self.validCC)
            #for s in ssplit: self.th.append_period(s)
            vpSplit = [self.pos_split(v,"VP",desiredCC=self.validCC) for v in ssplit]
            vpSplit = [item for sublist in vpSplit for item in sublist]#Flatten list
            npSplit = [self.pos_split(s,"NP",desiredCC=self.validCC) for s in vpSplit]
            flatSplit = [item for sublist in npSplit for item in sublist]#Flatten list
        except NoRightSibling:
            sys.stderr.write("No right sibling found for tree on CC, should be impossible. Check parse.")
            raise
        except NoLeftSibling:
            sys.stderr.write("No left sibling found for tree on CC, should be impossible. Check parse.")
            raise        
        except UnlevelCCSiblings:
            sys.stderr.write("Unlevel CC parse. Check parse.")
            raise
        for item in flatSplit: self.th.depth_ulid_deaugment(item)
        return flatSplit
