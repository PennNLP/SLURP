'''
Created on Oct 11, 2013
Define custom exceptions for semantics

@author: taylor
'''

class NoSChildVP(Exception):
    pass

class PosTooDeep(Exception):
    def __init__(self,head):
        return head

class VerbFrameCountError(Exception):
    #Raised if the Verb frame has too many or too few verbs
    pass

class SlotTreeCountError(Exception):
    #Raised if there are too many slots to be filled given a tree
    pass
