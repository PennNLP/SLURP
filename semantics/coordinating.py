'''
Created on Oct 18, 2013

@author: taylor
'''
from matchingTests import exampleCoordination
from matching import TreeHandler

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
        
    def vp_split(self,tree,count):
        '''The strategy for this is to find the CC in the VP (like the VB in matching),
            find the nearest left or right siblings and if they are heads, pop        
        '''
        res = []        
        cursor = [-1]
        for i in range(count):
            ccpath = self.th.get_main_pos_path(tree, "CC", -1, cursor=cursor)
            left = self.th.nearest_left_sibling(ccpath)
            right = self.th.nearest_right_sibling(ccpath, tree)    
            nporvp = self.th.which_parent(tree, ccpath, ["VP","NP"], -1)
            pos = nporvp.split(self.th.depthdelim)[0].split(self.th.posdelim)[0]
            if pos == "VP":
                res.append(ccpath)
                print 'VP CC: ',ccpath
                pass  
            else:
                print pos,' CC: ',ccpath
            cursor = ccpath
        return res
        
    
    def split_on_cc(self,tree):
        '''Split on "and"...extend to other CCs later.        
        '''
        ccs = [w in self.validCCs for w in tree.leaves()]
        if True not in ccs:
            return [tree]
        self.th.depth_ulid_augment(tree, 0)        
        vpSplit = self.vp_split(tree,ccs.count(True))
        #npSplit = [self.np_split(s) for s in vpSplit]
        #flatSplit = [item for sublist in npSplit for item in sublist]
        #return flatSplit
if __name__=="__main__":
    main()