'''
Created on Oct 11, 2013
Define custom exceptions for semantics

@author: taylor
'''

class NoSChildVP(Exception):
    pass

class PosTooDeep(Exception):
    #Given practical limits on tree depth, raise if POS is found but too deeply embedded 
    def __init__(self,pos):
        return pos

class VerbFrameCountError(Exception):
    #Raised if the Verb frame has too many or too few verbs
    pass

class SlotTreeCountError(Exception):
    #Raised if there are too many slots to be filled given a tree
    pass

class NoRightSibling(Exception):
    #Raised if operation over a tree contains no right sibling to a node
    pass

class TreeProcError(Exception):
    def __init__(self,e):
        return e
    
class NodeNotFound(Exception):
    #Raised if the node cannot be found in the tree
    pass

class SlotNotFilledError(Exception):
    #Raise if the slot cannot be filled
    pass
