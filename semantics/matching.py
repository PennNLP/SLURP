"""
Matches Verbnet frames to NLTK style syntax parse trees.
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

from semantics.treehandler import TreeHandler 
from _matchingExceptions import NoSChildVP, PosTooDeep, VerbFrameCountError,SlotTreeCountError, NoRightSibling, TreeProcError, NodeNotFound, SlotNotFilledError, NoSubjectFound, NoObjectFound
import sys
import copy
DEBUG = False

def main():
    """tests are in semantics/matchingTests"""


class ParseMatcher(object):
    """
    This class keeps track of state (traversal) information
    given a verbnet frame and syntax parse.
    strictMatching is a numeric value that sets the acceptable depth distance from the main VP
    strictPPMatching is a numeric value that sets the acceptable depth distance from the NP head
    """
    TO_TAG = "TO"
    AGENT_KEY = "Agent"
    DEFAULT_ROLE = "DEFAULT_ROLE"
    COMMA_TAG = ","
    pos_map = {         'S' : ['S'],
                        'DT' : ['DT'],
                        'ADV' : ['ADV','ADVP','RB'], 
                        'VP' :{"gerund":["VBG"]},
                        'VERB' : ['VB','VBP','VPNONE'],
                        'NP' : {"Agent" : ['NPNONE','NNS','NN','NNP','NNPS','PRP'],
                                "Theme" : ['NNS','NN','NNP','NNPS','PRP'],
                                DEFAULT_ROLE : ['NPNONE','NNS','NN','NNP','NNPS','PRP']},
                        'PREP' : {"to towards" : ["TO","IN"],
                                  "against into onto" : ["IN"],
                                  "to into" : ["IN","TO"],
                                  "PREP": ["IN","TO"],
                                  "in" : ["IN"],
                                  "to" : ["TO"],
                                  "at in with" : ["IN"],
                                  "as" : ["IN"],
                                  "with" : ["IN"],
                                  "for" : ["IN"],
                                  "from for on" : ["IN"],
                                  "by" : ["IN"],
                                  DEFAULT_ROLE : ["IN"]                                
                                  },
                        'LEX' : ['there'],
                        'ADJ' : ['ADJ']                        
            }   
    depth_map = {'maxVB' : -1,
                      'maxNONE' : -1,
                      'maxTO': -1,
                      'maxIN': -1,
                      'maxPREP': -1}
    #Put restrictions on slots    
    semantic_rules = {'slot_keys' :{
                                    1 :{
                                           'Theme' : {
                                                      'parentRestriction':{0 : 'NP',
                                                                           1: 'PP',                                                                                  
                                                                           }
                                                      }
                                             }
                                    }
                      }
    
    def __init__(self,smatch=0,sppmatch=2):
        self.strictMatching = smatch
        self.strictPPMatching = sppmatch
        self.th = TreeHandler()
        self.role_dict= {
                         'subject' : 1,
                         'verb' : 2,
                         'prep' : 3,
                         'object' : 4
                         }
        
    def check_rules(self,slot,path):
        """Check rules in semantic_rules given a slot and a path"""
        srules = self.semantic_rules['slot_keys']
        for idx in srules:
            for value in srules[idx]:
                if slot[idx] == value:
                    for rule in srules[idx][value]:
                        if rule == "parentRestriction":
                            child = srules[idx][value][rule][0]
                            eventualParent = srules[idx][value][rule][1]
                            if slot[0] == child:
                                return eventualParent not in [self.th.get_pos(w[0]) for w in path]
        return True

    @staticmethod
    def phrase_head(tree,phrase):
        phrase = phrase.split(TreeHandler.depthdelim)[0].split(TreeHandler.posdelim)[0]#Split dash-types off
        try:
            tree.node
        except AttributeError:
            if type(tree) == str: return None
            if DEBUG: print tree
        if phrase in ParseMatcher.pos_map:
            if type(ParseMatcher.pos_map[phrase]) == list:
                if tree.node in ParseMatcher.pos_map[phrase]:
                    return tree   
        for branch in tree:             
            res = ParseMatcher.phrase_head(branch,phrase)
            if res: return res
        
    def get_path(self,tree,slot,cursor=[-1]):
        """Return the path to the head of the slot"""        
        pos, role, secondary, tertiary = slot      
        if not pos in self.pos_map:
            # Verbframe slot pos is unknown
            return None
        heads = self.pos_map[pos]
        lexical_roles = None#For preps: to, towards, against etc...
        if type(heads) == type({}):
            if not role in heads:
                #Verbframe slot role for this pos is unknown
                heads = heads[self.DEFAULT_ROLE]
            else:
                heads = heads[role]
                if role.islower():
                    #lowercase role => we want the lexical_role to match the leaf
                    lexical_roles = role.split(" ")
        maxd = -1
        for head in heads:            
            if 'max'+head in self.depth_map:
                #set max depth if it exists
                maxd = self.depth_map['max'+head]        
        #mainpos = self.th.get_main_pos_path(tree,heads,maxd,cursor)
        mainpos = self.th.get_main_pos_phrasepath(tree,heads,maxd,cursor)
        #Check to see if the word is one of the roles
        if mainpos:
            if not self.check_rules(slot,mainpos): return None
            leaf = self.th.get_leaf(tree,[w[1] for w in mainpos])
            if lexical_roles and leaf.lower() in lexical_roles:
                return [w[1] for w in mainpos]
            elif lexical_roles:
                #We defined lexical_roles and the leaf was not one of them.
                return None
            return [w[1] for w in mainpos]
        else:
            return None
        if DEBUG : print 'path to mainpos for slot(',slot,') ',mainpos        
        return mainpos     
    
        
    def match_subject_object(self,left,right,v,tree):
        """    Match the object/subject given the subframes(left,right), v(path to VB) and tree(full tree)
                return [path] to each slot        
            @input v is the path to the VP + the last item is the VB branch
            @input tree is the VP itself
            @input subframe is the object subframe 
            object must be to the right of v
        """   
        stree = copy.deepcopy(tree)
        otree = copy.deepcopy(tree)        

        """Nearest siblings
            nearest sibling methods return inclusive of head so we can pop greater than the last elmt
        """
        sLeftBranch = self.th.nearest_left_sibling(v)
        sLeftSibling = copy.deepcopy(sLeftBranch)
        snext = 0
        if len(sLeftSibling) > 0:
            sLeftSibling[-1] -= 1   
            if self.th.get_pos(tree[sLeftSibling].node).startswith(self.TO_TAG):
                #This verb is in its infinitive form, therefore try checking one level up for subject
                sLeftBranch = sLeftBranch[:-1]
            elif self.th.get_pos(tree[sLeftSibling].node).startswith(self.COMMA_TAG):
                #This subject may end in a comma, check and see
                if sLeftBranch[-1] != 0:
                    sLeftBranch[-1] -= 1
            if len(sLeftBranch) < 1:
                raise NoSubjectFound(left)
            for i in sLeftBranch[:-1]:
                stree = stree.pop(i)
            else:
                #last item of sLeftBranch is the branch to the VP, 
                #subtract one and you get the nearest left branch            
                sLeftBranch[-1] -= 1
                stree = stree.pop(sLeftBranch[-1])
            s = self.sequential_match_slots(snext,left, sLeftBranch, stree)
        else:
            #No left sibling, return error for each slot left of the verb
            s = [(slot,SlotNotFilledError) for slot in left]
            

        oRightBranch = self.th.nearest_right_sibling(v,otree)    
        if len(oRightBranch) < 1:
            raise NoObjectFound(right)    
        
        for i in oRightBranch[:-1]:
            otree = otree.pop(i)
            
        """At this point,   otree should be the constituent VP of the main verb
                  and    stree should be the * branch to the left of the VP"""        
        
        onext = oRightBranch[-1] + 1
        
        
        o = self.sequential_match_slots(onext,right, oRightBranch[:-1], otree)
        return s,o
    
    def pp_acceptpath(self,prevpath,nextslot,next,path,tree):
        """Returns True if the path points to a leaf that is an acceptable distance from the most
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
        """
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
        """Match the slots in the subframe given the tree and base
            If the slot is for a preposition that accepts embedding, fill slots as if it were not embedded.
            Otherwise, increment the cursor as if it were the head but keep looking.
            
            @input base where are we right now  
        """
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
                    #If we found the pos tag of the slot      
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
        print 'subject: ',[(w[0],w[1],self.th.get_leaf(tree,w[1])) for w in s]
        #print 'subject: ',[(w,self.get_leaf(tree,w)) for w in s]
        print 'verb: ',self.th.get_leaf(tree,v)
        #print 'object: ',[(b,self.get_leaf(tree,b)) for b in o]
        print 'object: ',[(b[0],b[1],self.th.get_leaf(tree,b[1])) for b in o]
        
            
    def proximity_match_frame(self,v,center,frame,tree):
        """Given a v(tup) and center(int) match the frame around that path."""        
        left = frame[:center]#S
        #V
        right = frame[center+1:]#O
        s,o = self.match_subject_object(left,right,v,tree)
        if DEBUG: self.print_svo(s,v,o,tree)
        return [s,v,o]
    
    def frames_match_frame (self, svo, tree):
        """Format svo into the format frames expects:
            {'<POS>' : tree,...}        """
        s, v, o = svo
        res = {}
        for i in s:
            #If the slot is an NP, we want the whole phrase
            phrase, role, secondary, tertiary = i[0]
            path = i[1]
            if phrase == "NP":
                res[role] = tree[path[:-2]]
            else:
                res[role] = tree[path[:-1]]
        res['VERB'] = tree[v[:-1]]
        for i in o:
            #If the slot is an NP, we want the whole phrase
            phrase, role, secondary, tertiary = i[0]
            path = i[1]
            if phrase == "NP":
                res[role] = tree[path[:-2]]
            else:
                res[role] = tree[path[:-1]]
            #res[i[0][1]] = self.th.get_parent(tree,i[1])
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
            return False
        return True
        
    def match_frame(self,frame,parse):
        """Try to match the frame to the parse"""
        try:
            fmatch = None
            if self.invalid_pos(frame):
                return None            
            if not self.check_for_augment(parse):
                self.th.depth_ulid_augment(parse, 0)
            verbslots = [(i,w) for i,w in enumerate(frame) if w[0] == "VERB"]            
            if len(verbslots) > 1: raise VerbFrameCountError
            verbindex, verbslot = verbslots[0]
            verbpath = self.get_path(parse,verbslot)
            if not verbpath:
                return None
            pmatch = self.proximity_match_frame(verbpath,verbindex,frame,parse)
            s,v,o = pmatch
            if True in [(type(w[1])==type) for w in s]: 
                if DEBUG: sys.stderr.write("Error finding subject for frame. "+ str(s)+"\n")
                return None
            elif True in [(type(w[1])==type) for w in o]: 
                if DEBUG: sys.stderr.write("Error finding object for frame. "+ str(o)+"\n")
                return None
            if DEBUG: self.print_svo(pmatch[0],pmatch[1],pmatch[2],parse)
            self.th.depth_ulid_deaugment(parse)
            fmatch = self.frames_match_frame(pmatch,parse)
               
            #Custom rules for frames
            if fmatch and self.AGENT_KEY not in fmatch:
                #Do not match any frames with an agent
                return None          
        except PosTooDeep, pos:
            sys.stderr.write(pos+" found but too deep given max part-of-speech depth\n")
        except VerbFrameCountError:
            sys.stderr.write(str(frame)+" contains too many or too few verbs...error\n")            
        except SlotTreeCountError:
            #Not all the slots could be filled for this frame given this parse
            pass
        except NoObjectFound, obranch:
            sys.stderr.write("Could not find the object in this parse, for this branch: "+str(obranch))
        except NoSubjectFound, sbranch:
            sys.stderr.write("Could not find the subject in this parse, for this branch: "+str(sbranch))
        except:
            sys.stderr.write("Uncaught error in verbframe matching: "+str(frame)+str(parse))
            return None
        

                
        return fmatch       

if __name__=="__main__":
    main()
            
            
                
        
        
        