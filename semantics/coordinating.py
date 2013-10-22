'''
Created on Oct 18, 2013

@author: taylor
'''
from matchingTests import exampleCoordination
from matching import TreeHandler
import sys
from _matchingExceptions import NoRightSibling, NoLeftSibling, UnlevelCCSiblings
import copy
DEBUG = False
def main():
    d = exampleCoordination.exDict
    splitter = Split()    
    for sent in d:
        print "Splitting on cc for sent: ",d[sent]["sent"]
        splitter.split_on_cc(d[sent]["tree"])
        
class Split(object):
    validCCs = ["and"]
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
        path.append(-1)
        return path
           
        
    def vp_split(self,tree,count):
        '''The strategy for this is to find the CC in the VP (like the VB in matching),
            find the nearest left or right siblings and if they are heads, pop        
        '''
        res = []        
        cursor = [-1]
        for i in range(count):
            ccpath = self.th.get_main_pos_path(tree, "CC", -1, cursor=cursor)
            nporvp = self.th.which_parent(tree, ccpath, ["VP","NP"], -1)
            pos = nporvp.split(self.th.depthdelim)[0].split(self.th.posdelim)[0]
            if pos == "VP":
                left = self.sibling_cc_path(ccpath) 
                right = self.sibling_cc_path(ccpath,tree=tree);
                if len(left) != len(ccpath) != len(right):
                    raise UnlevelCCSiblings
                lefttree = copy.deepcopy(tree)
                self.th.pop_path_two(lefttree,right,ccpath)
                res.append(lefttree) #Copy and put left branch in results and keep going
                self.th.pop_path_two(tree, left,ccpath)#Pop for real, keep looking
            else:
                if DEBUG: print pos,' CC: ',ccpath
            cursor = ccpath  
        res.append(tree)               
        return res
    
    def np_split(self,tree):
        '''The strategy for this is to find the CC in the VP (like the VB in matching),
            find the nearest left or right siblings and if they are heads, pop        
        '''
        res = []        
        cursor = [-1]
        count = self.num_ccs(tree) 
        for i in range(count):
            ccpath = self.th.get_main_pos_path(tree, "CC", -1, cursor=cursor)
            nporvp = self.th.which_parent(tree, ccpath, ["VP","NP"], -1)
            pos = nporvp.split(self.th.depthdelim)[0].split(self.th.posdelim)[0]
            if pos == "NP":
                left = self.sibling_cc_path(ccpath) 
                right = self.sibling_cc_path(ccpath,tree=tree);
                if len(left) != len(ccpath) != len(right):
                    raise UnlevelCCSiblings
                lefttree = copy.deepcopy(tree)
                #Instead of popping, need to replace parent with correct branch or else change how I match frames
                self.th.pop_path_two(lefttree,right,ccpath)
                #self.th.replace_parent(lefttree,left)
                res.append(lefttree) #Copy and put left branch in results and keep going
                self.th.pop_path_two(tree, left,ccpath)#Pop for real, keep looking
            else:
                if DEBUG: print pos,' CC: ',ccpath
            cursor = ccpath  
        res.append(tree)               
        return res
        
    def num_ccs(self,tree):
        return [self.th.remove_ulid(w) in self.validCCs for w in tree.leaves()].count(True)

    def split_on_cc(self,tree):
        '''Split on "and"...extend to other CCs later.        
        '''
        numCCs = self.num_ccs(tree)
        if numCCs < 1:
            return [tree]
        self.th.depth_ulid_augment(tree, 0)
        try:        
            vpSplit = self.vp_split(tree,numCCs)
            npSplit = [self.np_split(s) for s in vpSplit]
            flatSplit = [item for sublist in npSplit for item in sublist]
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
if __name__=="__main__":
    main()