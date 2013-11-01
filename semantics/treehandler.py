"""
Handles intelligent tree manipulation and analysis.
"""

# Copyright (C) 2013 Taylor Turpen
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from semantics.tree import Tree
from _matchingExceptions import NoSChildVP, PosTooDeep, VerbFrameCountError,SlotTreeCountError, NoRightSibling, TreeProcError, NodeNotFound, SlotNotFilledError, NoSubjectFound, NoObjectFound
import sys
import copy
DEBUG = False

def main():
    '''tests are in semantics/matchingTests'''

class TreeHandler(object):
    depthdelim = "."
    posdelim = "-"
    nullnode = "NONE"
    accepted_conditions = ["SBAR-TMP","SBAR-ADV"]
    accepted_punc = [","]
    
    def __init__(self):
        #A unique leaf identifier in case there are identical leaves
        self.ulid = 0        
    
    def append_period(self,tree):
        if len(tree) > 0 and tree[-1].node != '.':
            node = '.'+self.depthdelim+str(1)
            children = ['.'+'__'+str(self.get_ulid())+'__']            
            tree[1].append(Tree(node,children))

            
    @staticmethod                
    def pop_sbars(tree):
        '''Given a tree pop any conditional sbars'''
        TreeHandler.pop_vp_sbars(tree)      
        for i, branch in enumerate(tree):            
            if branch.node in TreeHandler.accepted_conditions:
                if len(tree) > i and tree[i+1].node in TreeHandler.accepted_punc:                    
                    tree.pop(i); tree.pop(i)#pop twice for sbar and punc
                else:
                    tree.pop(i)
                    
    @staticmethod                    
    def pop_vp_sbars(tree):
        '''Given a tree pop the sbars in any VPs'''
        for i, branch in enumerate(tree):        
            if branch.node == "VP":    
                for j, vbranch in enumerate(branch):
                    if vbranch.node in TreeHandler.accepted_conditions:
                        tree[i].pop(j)#Pop once, no punc

        
                    
    def get_ulid(self):
        self.ulid += 1
        return self.ulid-1
    
    def remove_ulid(self,leaf):
        if type(leaf) == Tree and len(leaf) == 1 and type(leaf.node) == str:
            leaf = leaf[0]
        return leaf.split('__')[0]
    
    def pop_path(self,tree,path):
        if len(path) < 2:
            sys.stderr.write('tried to pop_path for a path with no length')
        elif len(path) == 2:
            tree.pop(path[0])
        else: 
            self.pop_path(tree[path[0]],path[1:])
            
    def pop_left(self,tree,path):
        '''Pops every left sibling of path, and then pops path. 
            E.x. "this, that, and the other" leaves only "the other"
        '''
        if len(path) < 3:
            keep = path
            keep[0] +=1
            for i in range(keep[0]):
                #if idx of keep is 2-> 3rd item, pop twice
                tree.pop(0)
        else: 
            self.pop_left(tree[path[0]],path[1:])
            if len(path) == 3 and len(tree[path[0]]) == 1:
                #If we created an only-child, bump the child up
                tree[path[0]] = tree[path[0]][0]
        
            
    def pop_path_cc(self,tree,lemma,cc):
        '''This works like pop_path but requires two paths of the same length
            Intended to be used for popping siblings of a conjunction.
        '''
        if len(lemma) == 1:
            sys.stderr.write('tried to pop_path_two on a path with no leaf (length==1)')
        elif len(lemma) == 2:
            #Spop            
            tree.pop()
            if lemma[0] > cc[0]:
                tree.pop(lemma[0])
                tree.pop(cc[0])
            else:
                tree.pop(cc[0])
                tree.pop(lemma[0])                
            if len(tree) == 1:
                #If only one child left, replace
                return tree[0]
#             else:
#                 sys.stderr.write('Tried to replace CC parent but too many children')
        elif len(lemma) == 3:
            if lemma[1] > cc[1]:
                tree[lemma[0]].pop(lemma[1])
                tree[cc[0]].pop(cc[1])
            else:
                tree[cc[0]].pop(cc[1])
                tree[lemma[0]].pop(lemma[1])                
            if len(tree[lemma[0]]) == 1:
                #If only one child left, 
                tree[lemma[0]] = tree[lemma[0]][0]
        elif lemma[0] != cc[0]:
            sys.stderr.write('pop_path_cc paths are not similar enough')                
        else: 
            self.pop_path_cc(tree[lemma[0]],lemma[1:],cc[1:])
            
    def replace_parent(self,tree,path):
        '''Replaces the parent of the last two digits of path with the path[-2] child'''
        if len(path) < 2:
            sys.stderr.write('tried to replace_parent for a path that is too short')
        elif len(path) == 2:
            tree = tree[path[0]]
        else:
            self.replace_parent(tree[path[0]],path[1:])                                     
            
    def get_leaf(self,tree,path):    
        if len(path) < 2:
           return self.remove_ulid(tree[path[0]])
        elif len(path) == 2:
            leaf = self.remove_ulid(tree[path[0]][path[1]])
            return leaf
        else: 
            return self.get_leaf(tree[path[0]],path[1:])   
        
    def get_parent(self,tree,path):
        if len(path) < 2:
            sys.stderr.write('tried to print_path for a leaf string or no path')
        elif len(path) == 2:
            tree[path[0]][path[1]] = self.remove_ulid(tree[path[0]][path[1]])
            return tree[path[0]]
        else: 
            return self.get_parent(tree[path[0]],path[1:])  
        
    def which_parent(self,tree,path,pos,curparent):
        '''Recursively finds the closest parent in path that is in pos given tree
        '''
        curpos, curdepth = self.node_pos(tree)
        if curpos in pos:
            curparent = tree.node         
        if len(path) < 2:
            sys.stderr.write('tried to print_path for a leaf string or no path')
        elif len(path) == 2:            
            return curparent
        else: 
            return self.which_parent(tree[path[0]],path[1:],pos,curparent)  

    def common_ancestor(self,tree,indexa,indexb):
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
            pos = self.get_pos(tree.node)
            if self.depthdelim in tree.node: return True#Tree has already been augmented
        except AttributeError:
            if DEBUG: print tree
        else:
            #augment tree
            if pos == "S" and depth != 0:
                tree.node = "S"#If we wanted to make subsentence S into SubS, this would be where we do it
            tree.node+=self.depthdelim+str(depth)
            if DEBUG: print '(',tree.node,
            for i,child in enumerate(tree):
                if type(child) == str:
                    tree[i] += '__'+str(self.get_ulid())+'__'
                else:
                    if child.node == self.posdelim+self.nullnode+self.posdelim:
                        #(NP (-NONE- *)) to (NP (-NPNONE- *))
                        child.node = self.posdelim+tree.node.split(self.depthdelim)[0].split(self.posdelim)[0]+self.nullnode+self.posdelim 
                self.depth_ulid_augment(child,depth+1)
            if DEBUG: print ')',    
            
    def depth_ulid_deaugment(self,tree):
        '''Recursive traversal that augments a tree with the depth of each node attached to each node
            And also appends a unique identifier to each node in case of multiple lemmas
        '''        
        try:
            tree.node
        except AttributeError:
            if DEBUG: print tree
        else:
            #deaugment tree
            if tree.node[0] != '.':
                tree.node = tree.node.split(self.depthdelim)[0]
            else: 
                tree.node = '.'
            if DEBUG: print '(',tree.node,
            for i,child in enumerate(tree):
                if type(child) == str:
                    #tree[i] += '__'+str(self.get_ulid())+'__'
                    tree[i] = tree[i].split('__')[0]                     
                self.depth_ulid_deaugment(child)
            if DEBUG: print ')',    
            
    def get_subtree(self,tree,path):
        '''Recursive traversal that returns the subtree when no more path'''
        if len(path) == 0:
            return tree
        self.get_subtree(tree[path[0]],path[1:])
        
    @staticmethod
    def node_pos(tree):
        depthsplit = tree.node.split(TreeHandler.depthdelim)#Split on depth delim
        possplit = depthsplit[0].split(TreeHandler.posdelim)#split on pos delim
        if len(possplit) > 1 and possplit[0] == '' and possplit[1] != '':
            return possplit[1], depthsplit[-1]        
        return possplit[0],depthsplit[-1]   
    
    @staticmethod
    def get_pos(node):
        depthsplit = node.split(TreeHandler.depthdelim)#Split on depth delim
        possplit = depthsplit[0].split(TreeHandler.posdelim)#split on pos delim
        return possplit[0]
             
        
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
                if npos != '' and npos in pos: 
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
                    if len(cursor) == 1:
                        if i < len(tree)-1: 
                            sys.stderr.write("More branches in this tree but returning none because of cursor...check that initial cursor is correct. It should probably be longer than it is now.")  
                        return None#We shouldn't get here when there are more branches
                    #if len(cursor) == 1: cursor = [-1]                                         
                    res = self.leftmost_pos(branch,pos,cursor[1:])
                    if res:
                        return res                
                else:
                    res = self.leftmost_pos(branch,pos,cursor)
                    if res:
                        return res                    
        return None
        
    def get_main_pos_path(self,tree,pos,maxPosDepth,cursor=[-1]):
        '''Return path to the shallowest leftmost pos'''      
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
    
    def get_main_pos_phrasepath(self,tree,pos,maxPosDepth,cursor=[-1]):
        '''Return path to the shallowest leftmost pos, with phrases at each branch'''      
        subtree = self.leftmost_pos(tree, pos,cursor)
        if not subtree:
            return False
        leaves = subtree.leaves()
        index = leaves.index(subtree[0])
        path = self.get_phrasepath_to_subtree(tree,subtree)
        if maxPosDepth != -1 and len(path) > maxPosDepth:            
            raise PosTooDeep(pos)        
        return path 
    
    def get_phrasepath_to_subtree(self,tree,subtree):
        '''Recursively returns the path to the node
            ex: [('S', 1), ('VP', 0), ('VB', 0)] 
            s.t. from the S node, take the 1 branch, from the vp node take the 0 path and from the VB node take the 0 path (leaf str) 
        '''
        try:
            tree.node
        except AttributeError:
            return None#This is a leaf
        if tree == subtree: return [(tree.node,0)]        
        for i, branch in enumerate(tree):
            res = self.get_phrasepath_to_subtree(branch,subtree)
            if res:
                res.insert(0,(tree.node,i))
                return res        
        return None
            
    
    def nearest_right_sibling(self,leafpath,tree):
        '''Returns the path(inclusive of the first branch in leafpath past the parent)
            to the parent of the nearest right sibling of leafpath
            (1,0,1,1,0,0) returns [1,0,1,1,0] iff the parent(subtree) at [1,0,1,1] has > 1 children
            destructive mthd, makes a copy of tree            
        '''
        #destructive mthd, make copy
        tree = copy.deepcopy(tree)                
        path = [(-1,tree)]#initialize path        
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
            #Not sure why i put this conditional in...
            #elif not foundRSibling and branch > 0:            
                #raise NoRightSibling 
            if foundRSibling:                  
                if len(parentpath) == 0:
                    #returned path is inclusive of final branch to leafpath, for first item append that to the result   
                    parentpath.insert(0,last[0])                    
                if branch != -1:
                    #If this is not the root node, insert
                    parentpath.insert(0,branch)     
            last = (branch,r)        
        if len(parentpath) < 1:
            raise NoRightSibling  
        else:
            return parentpath        
                
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
    
if __name__=="__main__":
    main()
            
            
                
        
        
        