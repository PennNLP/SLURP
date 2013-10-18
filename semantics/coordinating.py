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
        splitter.split_on_cc(d[sent]["tree"])
        
class Split(object):
    def __init__(self):
        self.th = TreeHandler()        
        
    def vp_split(self,tree):
        self.th.depth_ulid_augment(tree, 0)
        ccpath = self.th.get_main_pos_path(tree, "CC", -1)
        print 'CC: ',ccpath
    
    def split_on_cc(self,tree):        
        vpSplit = self.vp_split(tree)
        allSplit = [self.np_split(s) for s in vpSplit]
        flatSplit = [item for sublist in allSplit for item in sublist]
        return flatSplit
if __name__=="__main__":
    main()