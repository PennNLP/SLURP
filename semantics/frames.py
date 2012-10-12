"""Frames.py

Ian Perera

Converts parse tree representation into a tree that can be matched to Verbnet
frames, and then returns the matching frames and their corresponding trees.
"""
# -*- coding: iso-8859-1 -*-
import re
import pickle
import os
from tree import Tree
from xml.etree.ElementTree import parse
from copy import deepcopy
from collections import defaultdict


WORD_SENSE_FILENAME = 'word_sense_mapping.pkl'
MODULE_PATH = os.path.dirname(os.path.abspath(__file__))
VERBNET_DIRECTORY = os.path.join(MODULE_PATH, 'Verbnet', 'verbnet-3.1')


def load_word_sense_mapping():
    """Loads the pickle file for mapping words to VerbNet frames. *Required*
    for other functions to work."""
    
    word_sense_path = os.path.join(VERBNET_DIRECTORY, WORD_SENSE_FILENAME)
    try:
        # Try to open the pickled file, allowing us to fail fast if 
        # there's no pickle file
        word_sense_file = open(word_sense_path, 'r')
        
        # Otherwise, confirm that the pickled file is up to date
        # Get the most recent file in the directory
        max_mtime = 0
        for dirname, _, files in os.walk(VERBNET_DIRECTORY):
            for filename in files:
                full_path = os.path.join(dirname, filename)
                try:
                    mtime = os.stat(full_path).st_mtime
                except (OSError, IOError):
                    continue
                if mtime > max_mtime:
                    max_mtime = mtime
                    max_file = filename
        # Make sure the pickle file is most recent
        if max_file != WORD_SENSE_FILENAME:
            raise IOError
        
        # Load the file if all is well
        result = pickle.load(word_sense_file)
    except IOError:
        print "Word sense pickle file is missing or out of date, creating it..."
        result = generate_mapping(word_sense_path)

    return result

def generate_mapping(result_filename):
    
    file_list = os.listdir(VERBNET_DIRECTORY)
    temp_word_sense_mapping = defaultdict(set)

    for filename in file_list:
        if (filename[-4:] == '.xml'):
            try:
                with open(os.path.join(VERBNET_DIRECTORY, filename), 'r') as f:
                    tree = parse(f)
                    root_element = tree.getroot()
                    class_id = root_element.attrib['ID']
                    for member in tree.getroot().findall('MEMBERS/MEMBER'):
                        temp_word_sense_mapping[member.attrib['name']].\
                                                    add((filename,class_id))
    
                    for subclass in tree.getroot().findall('SUBCLASSES/VNSUBCLASS'):
                        class_id = subclass.attrib['ID']
                        for member in subclass.findall('MEMBERS/MEMBER'):
                            temp_word_sense_mapping[member.attrib['name']].\
                                                    add((filename,class_id))
            except IOError:
                continue
                    

    with open(result_filename, 'w') as f:
        pickle.dump(temp_word_sense_mapping, f)

    return temp_word_sense_mapping


word_sense_mapping = load_word_sense_mapping()

# Mapping from Verbnet tags to Treebank tags
tag_mapping = {'NP' : ['NP'],
               ('NP', 'Location') : ['PP-LOC', 'PP-DIR', 'PP-CLR', 'NN', \
                                     'NP-A', 'WHADVP','ADVP-TMP'],
               ('NP', 'Destination') : ['PP-LOC', 'PP-DIR', 'NN', 'NP-A',\
                                        'ADVP', 'PP-CLR', 'WHADVP'],
               ('NP', 'Asset') : ['NP-A'],
               ('NP', 'Agent') : ['NP-SBJ-A', 'NP', 'NP-A'],
               ('NP', 'Beneficiary') : ['NP-A'],
               ('NP', 'Recipient') : ['NP-A'],
               ('NP', 'Patient') : ['NP'],
               ('NP', 'Instrument') : ['NP-A'],
               ('NP', 'Topic') : ['S-A', 'NP-A', 'WHNP'],
               ('NP', 'Theme') : ['NP-A', 'NP-SBJ-A', 'NP', 'NP-SBJ', \
                                  'WHNP'],
               ('PREP', ) : ['exact'],
               'PREP' : ['IN','TO','ADVP-DIR'],
               'VERB' : ['VB']
               }

# Mapping from syntax restrictions to Treebank tags and matching conditions
synrestr_mapping = {'to_be' : ('NP', 'to be', 'to_be','begins'),
                    'ac_ing' : ('VP', 'gerund', 'ac_ing', 'VBG'),
                    'that_comp' : ('S', 'that', 'that_comp', 'begins'),
                    'wh_comp' : ('S', 'what', 'wh_comp', 'begins'),
                    'poss_ing' : ('S', 'wanting', 'poss_ing', 'VBG'),
                    'wh_inf' : ('S', 'how', 'wh_inf', 'WH')
                    }

auxiliary_verbs = ['was', 'is', 'get', 'are', 'got', 'were', 'been', 'being']
        
class VerbFrameObject:
    """Object which contains elements of a frame in a list. Each element has
    the form (POS tag, role, syntactic restriction, child node to match /
    matching condition)""" 
    def __init__(self, classid, verb, frame, example=""):
        self.classid = classid
        self.verb = verb
        self.frame = frame
        self.example = example
        self.frame_list = []

        # Populate the frame list with the frame elements
        for element in self.frame:
            role = element.tag
            synrestr = ''
            child_match = ''
            element_tuple = ()

            # Find any syntax restrictions
            for restr in element.findall('SYNRESTRS/SYNRESTR'):
                synrestr = restr.attrib['type']

            # Get the role of the frame element
            if 'value' in element.attrib.keys():
                role = element.attrib['value']
            
            element_tuple = (element.tag, role, synrestr, child_match)

            # If there is a syntax restriction, get the matching tuple
            if not synrestr == '' and synrestr in synrestr_mapping.keys():
                self.frame_list.append(synrestr_mapping[synrestr])
            else:
                self.frame_list.append(element_tuple)

    def __repr__(self):
        result = 'Classid: ' + str(self.classid) + ' Verb: ' + str(self.verb)

        return result
        

    def print_frame(self):
        for element in self.frame:
            if 'value' in element.attrib.keys():
                print str(element.tag) + " : " + str(element.attrib['value'])
            else:
                print str(element.tag)

    def match_parse(self, parse_tree):
        """Takes a Treebank parse tree compiled into NLTK's tree structure.
        Outputs a result dictionary mapping predicates to arguments"""
        result_dict = {}
        subtree_list = []
        subtrees = parse_tree.subtrees()

        # We need a list of subtrees, not a generator
        for subtree in subtrees:
            subtree_list.append(subtree)
            
        current_subtree = 0
        matches = 0

        # For each frame element, find the next subtree that matches
        for frame_tag in self.frame_list:
            # Go through the subtrees until you run out of subtrees
            while current_subtree < len(subtree_list):
                subtree = subtree_list[current_subtree]

                # If there is no explicit NP for the Agent role, insert one
                # NOTE: Assumes Agent role is the first in the frame, may not be true
                if (current_subtree == 0 and frame_tag[1] == 'Agent' and not
                        self.__match_subtree(subtree, frame_tag)):
                    result_dict[frame_tag[1]] = Tree("(NP-SBJ-A (-NONE- *))")
                    matches += 1
                    break


                if self.__match_subtree(subtree, frame_tag):
                    #print 'Match -> ' + str(frame_tag)
                    # If the subtree matches, add role->phrase to the dictionary
                    #result_dict[frame_tag[1]] = ' '.join(subtree.leaves())
                    result_dict[frame_tag[1]] = subtree
                    matches += 1
                    break

                current_subtree += 1

        if current_subtree == len(subtree_list):
            return None

        # Only return something if every frame was matched
        if matches == len(self.frame_list):
            return result_dict
        else:
            return None
        
    def __match_subtree(self, subtree, frame_tag):
        """Given a subtree, tries to match it to a frame element"""
        # Regex for POS tags        
        tag_match = re.compile(r'(?<=\()[A-Z][A-Z-]*')
        
        tags = tag_match.findall(str(subtree))


        # Check for special mapping/matching restrictions
        if frame_tag[2] == '':
            # Tag/Role -> Treebank map
            if ((frame_tag[0], frame_tag[1])) in tag_mapping.keys():
                tree_tags = tag_mapping[(frame_tag[0], frame_tag[1])]
            # PREP tag that requires exact match to preposition
            elif (not frame_tag[1] == '') and \
                 (frame_tag[0],) in tag_mapping.keys() and \
                 (not frame_tag[0] == frame_tag[1]) :
                tree_tags = tag_mapping[(frame_tag[0], )]
            # Regular VerbNet tag -> Treebank tag map
            elif frame_tag[0] in tag_mapping.keys():
                tree_tags = tag_mapping[frame_tag[0]]
            # No mapping needed
            else:
                tree_tags = [frame_tag[0]]
        else:
            tree_tags = [frame_tag[0]]

        if len(tags) > 0:
            for tree_tag in tree_tags:
                if tree_tag in tags[0]:
                    # Matched first word
                    if not frame_tag[3] == '':
                        # Now need to match a syntax restriction
                        
                        # Child Tag match required
                        if len(tags) > 1 and (not frame_tag[3] == '') and \
                           frame_tag[3] in tags[1]:
                            return True
                        # Beginning word match required
                        elif len(tags) > 1 and frame_tag[3] == 'begins':
                            prefix_match = re.compile(frame_tag[1])
                            if prefix_match.match(' '.join(subtree.leaves())):
                                return True
                    else:
                        return True
                # Exact word match required
                elif tree_tag == 'exact' and \
                ' '.join(subtree.leaves()).lower() in frame_tag[1].split():
                    return True

        return False

def activize_clause(parse_tree_clause):
    """Converts passive clauses to active voice, substituting 'something' for the
    implicit agent."""

    sbj_match = re.compile(r'NP-SBJ.*')

    sbj_position = None
    null_obj_position = None
    aux_verb_position = None
        
    for position in parse_tree_clause.treepositions():

        subtree = parse_tree_clause[position]

        if isinstance(subtree, Tree) and \
           sbj_match.match(subtree.node) is not None:
            sbj_position = position
        elif isinstance(subtree[0], Tree) and subtree[0] and subtree[0][0] == '*':
            null_obj_position = position
        if isinstance(subtree, Tree) and subtree[0] in auxiliary_verbs:
            aux_verb_position = position

    if sbj_position is not None and null_obj_position is not None and \
       aux_verb_position is not None:
        parse_tree_clause[null_obj_position][0][0] = 'something'
        sbj = deepcopy(parse_tree_clause[sbj_position])
        null_obj = deepcopy(parse_tree_clause[null_obj_position])
        parse_tree_clause[sbj_position] = null_obj
        parse_tree_clause[null_obj_position] = sbj
        parse_tree_clause[aux_verb_position] = ''
        

    return parse_tree_clause

def wh_movement(parse_tree):
    """Moves the WH cluase to the null element. Where are the hostages -> The hostages are where?"""
    wh_position = None
    null_position = None
    
    for position in parse_tree.treepositions():
        if not isinstance(parse_tree[position], Tree) or position == ():
            continue

        # Find the WH and null positions
        if parse_tree[position].node[:2] == 'WH':
            wh_position = position
        elif isinstance(parse_tree[position][0], Tree) and \
             parse_tree[position][0][0] == '*T*':
            null_position = position

        if wh_position is not None and null_position is not None:
            break

    # Move the WH branch
    if wh_position is not None and null_position is not None:
        parse_tree[null_position] = parse_tree[wh_position]
        del parse_tree[wh_position]

    return parse_tree            

def existential_there_insertion(parse_tree):
    """Insert an NP into an existential there node."""
    existential_there_position = None
    np_predicate_position = None
    
    for position in parse_tree.treepositions():
        # Don't check the leaves or the root
        if not isinstance(parse_tree[position], Tree) or position == ():
            continue

        ex_children = [child for child in parse_tree[position]
                    if isinstance(child, Tree) and child.node == 'EX']

        if len(ex_children) > 0:
            existential_there_position = position
            continue

        if parse_tree[position].node == 'NP' or 'NP-PRD' in parse_tree[position].node:
            np_predicate_position = position

    if existential_there_position is not None and \
       np_predicate_position is not None:
        parse_tree[existential_there_position] = \
                            parse_tree[np_predicate_position]
        del parse_tree[np_predicate_position]

    return parse_tree

def invert_clause(parse_tree):
    """Inverts clauses to enforce a NP-V ordering."""
    inverted = False
    vp_position = None
    np_position = None

    # Find the inverted clause tags and NP-V tags
    for position in parse_tree.treepositions():
        # We only want nodes, not leaves
        if not isinstance(parse_tree[position], Tree):
            continue

        if parse_tree[position].node == 'SINV' or \
           parse_tree[position].node == 'SQ':
            inverted = True
        
        children = [child for child in parse_tree[position]
                        if isinstance(child, Tree)]

        child_count = 0
        temp_np_position = None
        temp_vp_position = None
        
        for child in children: 
            if temp_np_position is None and child.node[:2] == 'NP':
                temp_np_position = child_count
            elif temp_vp_position is None and child.node[:1] == 'V':
                temp_vp_position = child_count
            child_count += 1

        if temp_np_position is not None and temp_vp_position is not None:
            vp_position = temp_vp_position
            np_position = temp_np_position
            break

    if position is not None:
        # Invert the NP-V ordering
        if inverted is True and vp_position is not None and np_position is not None:
            temp = parse_tree[position][vp_position]
            parse_tree[position][vp_position] = \
                                    parse_tree[position][np_position]
            parse_tree[position][np_position] = temp

        # Insert the PP into a node with a PRD tag
        prd_position = None
        pp_position = None

        if not isinstance(parse_tree[position], Tree):
            return parse_tree

        for sinv_position in parse_tree[position].treepositions():
            if not isinstance(parse_tree[position][sinv_position], Tree) or\
               sinv_position == ():
                continue
        
            if 'PRD' in parse_tree[position][sinv_position].node:
                prd_position = sinv_position
            elif 'PP' in parse_tree[position][sinv_position].node:
                pp_position = sinv_position

        if inverted is True and prd_position is not None and \
           pp_position is not None:
            parse_tree[position][prd_position] = \
                                    parse_tree[position][pp_position]
            del parse_tree[position][pp_position]

    return parse_tree

def split_conjunctions(parse_tree):
    """Find conjunctions in a given parse_tree, and create a list of parse trees
    with the conjunctions removed and the conjoined clauses in place of the
    parent node of the conjunction."""
    split_tree_list = []
    conjunction_dict = {}
    
    for position in parse_tree.treepositions():
        # We only want nodes, not leaves
        if not isinstance(parse_tree[position], Tree) or position == ():
            continue
        
        children = [child for child in parse_tree[position]
                    if isinstance(child, Tree)]

        # Get the tags to check for CC's or IN's
        children_nodes = [child.node for child in children]

        phrase_list = []
        conjunction = ''

        # Get any phrases that are siblings of a CC
        if 'CC' in children_nodes:
            for child in children:
                if not child.node == 'CC':
                    phrase_list.append(child)
                else:
                    conjunction = child[0]
    
        
        # Copy the tree and replace the parent node of the CC with the phrase
        # for each sibling of the CC
        for phrase in phrase_list:
            temp_tree = parse_tree.copy(deep=True)
            temp_tree[position] = phrase
            split_tree_list.append(temp_tree)
            
        if not conjunction == '':  
            conjunction_dict[position] = (split_tree_list, conjunction)

    if len(conjunction_dict.keys()) == 0:
        return {(0) : ([parse_tree], '') }

    return conjunction_dict

def split_clauses(parse_tree):
    """Splits clauses to parse them independently. Returns a dictionary with
    the positions of the split as the keys and the split trees as the values."""
    clause_dict = {}
    clause_match = re.compile(r'\(*\s*\(S[A-Z]*')
    sbar_match = re.compile(r'\(SBAR[ \-]')
    deletion_list = []

    for position in parse_tree.treepositions():
        # We only want nodes, not leaves
        if not isinstance(parse_tree[position], Tree):
            continue

        subtree = parse_tree[position]

        # Return subtrees where the root node is an S node and there are no
        # other S nodes in the subtree
        if sbar_match.match(str(subtree)) and len(subtree) > 1:
            clause_dict[position] = (subtree[1], subtree[0].leaves())
            deletion_list.append(position)

    # Remove the clause from the tree
    for position in deletion_list:
        del parse_tree[position]

    clause_dict[(0)] = (parse_tree, '')
    
    # Code to split S nodes, not always what we want
##    for position in parse_tree.treepositions():
##        if not isinstance(parse_tree[position], Tree):
##            continue
##
##        subtree = parse_tree[position]
##        
##        if subtree.node == 'S':
##            clause_dict[position] = (subtree, ' ')
            
    return clause_dict


def create_VerbFrameObjects(word):
    """Gets VerbNet data without using NLTK corpora browser."""
    vnclass_elements = get_vnclass_elements(word)

    vfo_list = []
    for element in vnclass_elements:
        for frame in element.findall('FRAMES/FRAME/SYNTAX'):
            vfo_list.append(VerbFrameObject(element.attrib['ID'],
                                            element,
                                            list(frame)))

    return vfo_list
        
    
def get_vnclass_elements(word):
    """Get all of the VerbNet classes for a given word."""
    class_id_filenames = word_sense_mapping[word]
    vnclass_elements = []
    for (cif, cid) in class_id_filenames:
        with open(os.path.join(VERBNET_DIRECTORY, cif), 'r') as xml_file:
            verb_xml = parse(xml_file)
            if verb_xml.getroot().attrib['ID'] == cid:
                vnclass_elements.append(verb_xml.getroot())

            for subclass in verb_xml.getroot().findall('SUBCLASSES/VNSUBCLASS'):
                if subclass.attrib['ID'] == cid:
                    vnclass_elements.append(subclass)

    return vnclass_elements


def pick_best_match(match_list):
    """From the list of tuples, with the first element being the dict containing role assignments,
    and the second element being the verb class, pick the match that has the most unique matches."""
    max_unique_matches = 0
    best_match = None
    best_match_sense = None
    for (match, sense) in match_list:
        unique_matches = len(set([' '.join(tree.leaves()) for tree in \
                                            match.values()]))
        if unique_matches > max_unique_matches:
            max_unique_matches = unique_matches
            best_match = match
            best_match_sense = sense

    return (best_match, best_match_sense)

def split_sentences(parse_tree_string):
    """Split the parse tree string into a separate tree string for each sentence."""
    open_brackets = 0

    split_parse_strings = []
    current_sentence = ''
    for letter in parse_tree_string:
        if letter == '(':
            open_brackets += 1
        elif letter == ')':
            open_brackets -= 1

        current_sentence += letter

        if open_brackets == 0 and not current_sentence.isspace():
            split_parse_strings.append(current_sentence)
            current_sentence = ''

    return split_parse_strings

def is_question(tree_string):
    """Given a parse tree string, return whether it is a question.""" 
    return ('WHADVP' in tree_string or \
        'WHNP' in tree_string or \
        'WHADJP' in tree_string or \
        'WHPP' in tree_string or \
        'SQ' in tree_string or \
        'SINV' in tree_string or \
        'SBARQ' in tree_string)

def get_wh_question_type(tree_string):
    """Given a parse tree string, return the type of wh-question it is."""
    if 'WHADVP' in tree_string or \
        'WHNP' in tree_string or \
        'WHADJP' in tree_string or \
        'WHPP' in tree_string:
        tree_string_lower = tree_string.lower()
        if 'where' in tree_string_lower:
            return 'Location'
        if 'status' in tree_string_lower or 'doing' in tree_string_lower:
            return 'Status'
        if 'who' in tree_string_lower:
            return 'People'
        if 'what' in tree_string_lower:
            return 'Entity'

    return None

def is_existential(tree_string):
    """Returns whether there is an existential there in the tree string."""
    return ('EX' in tree_string)

def is_yn_question(tree_string):
    """Given a parse tree string, return whether it is a yes-no question."""
    return ('SQ' in tree_string or \
        'SINV' in tree_string or \
        'SBARQ' in tree_string or \
        '?' in tree_string)

            
