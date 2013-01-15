# -*- coding: iso-8859-1 -*-
import re
import sys
import pickle
import os
from nltk.stem.wordnet import WordNetLemmatizer
from nltk.tree import Tree
from xml.etree.ElementTree import parse
from copy import deepcopy
from SemanticRepresentations import EnvironmentStatement, EnvironmentQuery, \
                                     StatusQuery, Command

verbnet_directory = '../Verbnet/verbnet-3.1/'

def load_word_sense_mapping():
    """Loads the pickle file for mapping words to VerbNet frames. *Required*
    for other functions to work."""
    current_directory = os.getcwd()
    os.chdir(verbnet_directory)
    
    with open('word_sense_mapping.pkl', 'r') as f:
        result = pickle.load(f)

    os.chdir(current_directory)
    return result


word_sense_mapping = load_word_sense_mapping()

lemmatizer = WordNetLemmatizer()

# Mapping from Verbnet tags to Treebank tags
tag_mapping = {'NP' : ['NP'],
               ('NP', 'Location') : ['PP-LOC', 'PP-DIR', 'PP-CLR', 'NN', \
                                     'NP-A', 'ADVP', 'WHADVP'],
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
               'PREP' : ['IN','TO'],
               'VERB' : ['VB']
               }

# Mapping from syntax restrictions to Treebank tags and matching conditions
synrestr_mapping = {'to_be' : ('NP','to be','to_be','begins'),
                    'ac_ing' : ('VP','gerund','ac_ing','VBG'),
                    'that_comp' : ('S','that','that_comp','begins'),
                    'wh_comp' : ('S','what','wh_comp','begins'),
                    'poss_ing' : ('S','wanting','poss_ing','VBG'),
                    'wh_inf' : ('S','how','wh_inf','WH')#,
                    #'oc_to_inf': ('S','to','oc_to_inf','begins')
                    }

auxiliary_verbs = ['was', 'is', 'get', 'are', 'got', 'were', 'been', 'being']
        
class Action:
    def __init__(self, parameters):
        self.parameters = parameter_dict

class VerbFrameObject:
    """Object which contains elements of a frame in a list. Each element has
    the form (POS tag, role, syntactic restriction, child node to match /
    matching condition)""" 
    def __init__(self,classid,verb,frame,example=""):
        self.classid = classid
        self.verb = verb
        self.frame = frame
        self.example = example
        self.frame_list = []

        # Populate the frame list with the frame elements
        for element in self.frame:
            tag = ''
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

    def print_frame(self):
        for element in self.frame:
            if 'value' in element.attrib.keys():
                print str(element.tag) + " : " + str(element.attrib['value'])
            else:
                print str(element.tag)

    def match_parse(self,parse_tree):
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
            #print 'Frame tag: ' + str(frame_tag)
            # Go through the subtrees until you run out of subtrees
            while current_subtree < len(subtree_list):
                subtree = subtree_list[current_subtree]
                #print subtree

                if self.__match_subtree(subtree, frame_tag):
                    #print 'Match -> ' + str(frame_tag)
                    # If the subtree matches, add role->phrase to the dictionary
                    result_dict[frame_tag[1]] = ' '.join(subtree.leaves())
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
        # Regex for words
        word_match = re.compile(r' [\w]*')
        
        tags = tag_match.findall(str(subtree))


        # Check for special mapping/matching restrictions
        if frame_tag[2] == '':
            # Tag/Role -> Treebank map
            if ((frame_tag[0],frame_tag[1])) in tag_mapping.keys():
                tree_tags = tag_mapping[(frame_tag[0],frame_tag[1])]
            # PREP tag that requires exact match to preposition
            elif (not frame_tag[1] == '') and \
                 (frame_tag[0],) in tag_mapping.keys() and \
                 (not frame_tag[0] == frame_tag[1]) :
                tree_tags = tag_mapping[(frame_tag[0],)]
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
    s_match = re.compile(r'S.*')
    sbj_match = re.compile(r'NP-SBJ.*')

    sbj_position = None
    null_obj_position = None
    aux_verb_position = None
        
    for position in parse_tree_clause.treepositions():

        subtree = parse_tree_clause[position]

        if isinstance(subtree, Tree) and \
           sbj_match.match(subtree.node) is not None:
            sbj_position = position
        elif isinstance(subtree[0], Tree) and subtree[0][0] == '*':
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

def modify_tags(parse_tree):
    """Change tags to fit the Verbnet tags better, making some domain
    assumptions."""

    spatial_prepositions = ['under','over','in','on','above','across',
                            'at','behind','below','beneath','beside',
                            'between','beyond','by','inside','into',
                            'near','outside','over','past','through',
                            'underneath','upon','via','within']

def wh_movement(parse_tree):
    wh_position = None
    null_position = None
    
    for position in parse_tree.treepositions():
        if not isinstance(parse_tree[position], Tree) or position == ():
            continue

        if parse_tree[position].node[:2] == 'WH':
            wh_position = position
        elif isinstance(parse_tree[position][0], Tree) and \
             parse_tree[position][0][0] == '*T*':
            null_position = position

        if wh_position is not None and null_position is not None:
            break

    if wh_position is not None and null_position is not None:
        parse_tree[null_position] = parse_tree[wh_position]
        del parse_tree[wh_position]

    return parse_tree
            

def existential_there_insertion(parse_tree):
    """Insert an NP into an existential there node."""
    existential_there_position = None
    np_predicate_position = None
    
    for position in parse_tree.treepositions():
        if not isinstance(parse_tree[position], Tree) or position == ():
            continue

        ex_children = [child for child in parse_tree[position]
                    if isinstance(child, Tree) and child.node == 'EX']

        if len(ex_children) > 0:
            existential_there_position = position
            continue

        if parse_tree[position].node == 'NP':
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
            #print str(child_count) + '->' + str(child.node)  
            if temp_np_position is None and child.node[:2] == 'NP':
                temp_np_position = child_count
            elif temp_vp_position is None and child.node[:1] == 'V':
                temp_vp_position = child_count
            child_count += 1

        if temp_np_position is not None and temp_vp_position is not None:
            vp_position = temp_vp_position
            np_position = temp_np_position
            break

    # Invert the NP-V ordering
    if inverted is True and vp_position is not None and np_position is not None:
        temp = parse_tree[position][vp_position]
        parse_tree[position][vp_position] = \
                                parse_tree[position][np_position]
        parse_tree[position][np_position] = temp

    # Insert the PP into a node with a PRD tag
    prd_position = None
    pp_position = None

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
    current_directory = os.getcwd()
    os.chdir(verbnet_directory)
    class_id_filenames = word_sense_mapping[word]
    vnclass_elements = []
    for (cif,cid) in class_id_filenames:
        with open(cif, 'r') as f:
            verb_xml = parse(f)
            if verb_xml.getroot().attrib['ID'] == cid:
                vnclass_elements.append(verb_xml.getroot())

            for subclass in verb_xml.getroot().findall('SUBCLASSES/VNSUBCLASS'):
                if subclass.attrib['ID'] == cid:
                    vnclass_elements.append(subclass)

    os.chdir(current_directory)
    return vnclass_elements
            

def extract_frames_from_parse(parse_tree_string):
    """Take a string representing the parse tree as input, and print the
    semantic parse. The result list consists of a list of tuples, with each
    tuple containing the VerbNet frame and its associated tree."""
    parse_tree = Tree.parse(parse_tree_string)
    #parse_tree.draw()

    split_clause_dict = split_clauses(parse_tree)

    for key, (clause, conjunction) in split_clause_dict.items():
        activized_clause = activize_clause(clause)
        split_clause_dict[key] = (activized_clause, conjunction)

    result_list = []
        
    for position, (clause, conjunction) in split_clause_dict.items():
        split_tree_dict = split_conjunctions(clause)
        
        if conjunction != '':
            result_list.append(conjunction)
        
        for split, (split_tree, conjunction) in split_tree_dict.items():
            if conjunction != '':
                result_list.append(conjunction)

            for tree in split_tree:
                tree = existential_there_insertion(tree)
                tree = invert_clause(tree)
                tree = wh_movement(tree)

                tree.draw() 

                # Regex for finding verbs 
                verb_finder = re.compile(r'(?<=VB[ DGNPZ]) *\w*(?=\))')

                # Get the lemma of the verb for searching verbnet
                verbs = (word.strip().lower() for word in
                         verb_finder.findall(str(tree)))
                
                for verb in verbs:
                    
                    lemmatized_verb = lemmatizer.lemmatize(verb,'v')
                    vfo_list = create_VerbFrameObjects(lemmatized_verb)

                    match_list = []
                    
                    for vfo in vfo_list:
                        match = vfo.match_parse(tree)
                        
                        if match:
                            match_list.append(match)

                    best_match = pick_best_match(match_list)
                    if not best_match is None:
                        result_list.append((best_match, tree))
                    

    return result_list

def pick_best_match(match_list):
    """From the list of dicts containing role assignments, pick the match that
    has the most unique matches."""
    score_list = []
    max_unique_matches = 0
    best_match = None
    for match in match_list:
        unique_matches = len(set(match.values()))
        if unique_matches > max_unique_matches:
            max_unique_matches = unique_matches
            best_match = match

    return best_match

def split_sentences(parse_tree_string):
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
    if 'WHADVP' in tree_string or \
        'WHNP' in tree_string or \
        'WHADJP' in tree_string or \
        'WHPP' in tree_string or \
        'SQ' in tree_string or \
        'SINV' in tree_string or \
        'SBARQ' in tree_string:
        return True
    else:
        return False

def get_stupid_semantics_from_parse_tree(parse_tree_string):
    """Aside from being stupid, there is a bug where split trees are being put in the
    same output. Temporarily use a set to fix this.

    No longer used."""
    parse_tree = Tree.parse(parse_tree_string)
    result_list = []
    final_conjunction = ''
    split_clause_dict = split_clauses(parse_tree)
    out_list = []
    inp_list = []
    saw = ''
    room_number_match = re.compile(r'\d')

    result_string = ''
        
    for position, (clause, conjunction) in split_clause_dict.items():
        split_tree_dict = split_conjunctions(clause)
        if len(conjunction) > 0: 
            final_conjunction = str(conjunction[0])
        
        for split, (split_tree, conjunction) in split_tree_dict.items():
            if len(conjunction) > 0:
                final_conjunction = str(conjunction[0])
                
            for tree in split_tree:
                # Regex for finding verbs 
                verb_finder = re.compile(r'(?<=VB[ DGNPZ]) *\w*(?=\))')
    
                # Get the lemma of the verb for searching verbnet
                verbs = (word.strip().lower() for word in
                         verb_finder.findall(str(tree)))
                lemmatizer = WordNetLemmatizer()

                for verb in verbs:
                    lemmatized_verb = lemmatizer.lemmatize(verb,'v')
                    lowercase_tree_string = str(tree).lower()

                    if lemmatized_verb == 'see' or lemmatized_verb == 'find': 
                        if ' bomb' in lowercase_tree_string:
                            inp_list.append('Bomb')
                        elif ' hostage' in lowercase_tree_string:
                            inp_list.append('Hostage')
                        elif (' bad' in lowercase_tree_string and \
                             'guy' in lowercase_tree_string) or \
                             ' terrorist' in lowercase_tree_string:
                            inp_list.append('BadGuy')
                        elif ' me' in lowercase_tree_string:
                            inp_list.append('Commander')
                    elif lemmatized_verb == 'flip' or lemmatized_verb == 'turned':
                        inp_list.append('Flipped')
                    elif lemmatized_verb == 'wait':
                        inp_list.append('WaitCommand')
                        out_list.append('Wait')
                    elif lemmatized_verb == 'defuse' or lemmatized_verb == 'destroy':
                        out_list.append('Defuse')
                    elif lemmatized_verb == 'tell' or lemmatized_verb == 'call' or \
                         lemmatized_verb == 'notify':
                        out_list.append('Tell')
                    elif lemmatized_verb == 'go' or lemmatized_verb == 'move':
                        index = room_number_match.search(lowercase_tree_string)
                        if index:
                            room_number = lowercase_tree_string[index]
                            out_list.append('r' + room_number) 
                    for i, action in enumerate(out_list):
                        if action == 'Tell':
                            if saw != '':
                                out_list[i] += saw
                            else:
                                out_list[i] = 'CallCommander'

                    #print inp_list
                    #inp = inp_list[0]
                    for inp in inp_list:
                        if len(out_list) > 0 and not \
                           (final_conjunction != '' and inp == ''):
                            result_tuple = (inp,tuple(set(out_list)))
                            result_list.append(result_tuple)

    good_results = result_list

    if len(result_list) > 1:
        good_results = list(set([result for result in result_list if result[0]]))

    best_results = {}
    inputs = {}
    for result in good_results:
        inputs[result[0]] = 0

    for result in good_results:
        if len(result[1]) > inputs[result[0]]:
            best_results[result[0]] = result
            inputs[result[0]] = len(result[1])


    return best_results.values()

def get_question_semantics_from_parse_tree(parse_tree_string):
    """No longer used."""
    parse_tree = Tree.parse(parse_tree_string)
    result_string = ''
    
    for position in parse_tree.treepositions():
        # We only want nodes, not leaves
        if not isinstance(parse_tree[position], Tree):
            continue

        node = parse_tree[position].node
        children = [child for child in parse_tree[position]
                        if isinstance(child, Tree)]

        subtree_string = str(parse_tree[position])
        
        if 'WH' in node:
            if 'How' in subtree_string or 'how' in subtree_string and \
               'many' in subtree_string:
                result_string += 'number '
                for child in children:
                    if 'NN' in child.node:
                        if 'terrorist' in str(child[0]):
                            result_string += 'terrorist '
                        elif 'hostage' in str(child[0]):
                            result_string += 'hostage '
                        elif 'bomb' in str(child[0]):
                            result_string += 'bomb '
            elif 'Where' in subtree_string or 'where' in subtree_string:
                result_string += 'location '
            elif 'Who' in subtree_string or 'who' in subtree_string:
                result_string += 'people '
            elif 'What' in subtree_string or 'what' in subtree_string:
                if 'are' in subtree_string or 'is' in subtree_string:
                    result_string += 'action '
                elif not 'action' in result_string and not 'objects' in result_string:
                    result_string += 'objects '

        if 'SBARQ' in node:
            if ('What' in subtree_string or 'what' in subtree_string):
                if 'doing' in subtree_string:
                    result_string += 'action '
                elif 'in' in subtree_string and not 'objects' in result_string:
                    result_string += 'objects '

        if 'VP' in node:
            if 'Are' in subtree_string or 'are' in subtree_string or \
               'Is' in subtree_string or 'is' in subtree_string:
                if 'in' not in subtree_string:
                    result_string += 'is '     

        if 'NP' in node:
            if 'room' in subtree_string:
                result_string += 'room '
            if 'hallway' in subtree_string:
                result_string += 'hallway '
            if 'Junior' in subtree_string or 'you' in subtree_string:
                result_string += 'Junior '
            if 'bomb' in subtree_string:
                result_string += 'bomb '
            if 'hostage' in subtree_string:
                result_string += 'hostage '
                
        if 'CD' in node and 'room' in result_string:
                result_string += parse_tree[position][0]

    return result_string

def process_parse(parse_string_list):
    """Returns (string response, list[MetaPAR tuple])"""
    par_text = ''
    question_text = ''
    response = 'Ok!'
    par_list = []
    answer_list = []
    commanded = False
    
    for parse_tree_string in parse_string_list:
        if is_question(parse_tree_string):
            answer_list.append(get_question_semantics_from_parse_tree(parse_tree_string))
        else:
            par_list.extend(extract_frames_from_parse(parse_tree_string))
            commanded = True
    
    if commanded:
        answer_list.append('Ok!')

    return (answer_list, par_list)

def create_semantic_structures(frame_semantic_list):
    """Take in the list of VerbNet frames and generate the semantic
    representation structures from them."""
    semantic_representation_list = []

    print frame_semantic_list

    # Frame semantic list is a list of conjunction strings and tuples, where the
    # first element of the tuple is the frame semantics, and the second element is
    # the original tree branch it came from.
    for frame in frame_semantic_list:
        obj = None
        location = None
        quantifier = 'sd'
        verb= None
        number = None
        # Check that this is a VerbNet frame and not a conjunction
        try:
            frame_items = frame[0].items()
        except:
            frame_items = None
            semantic_representation_list.append(frame)
            continue
        
        for key, value in frame_items:
            if key == 'Theme' or key == 'Patient' or key == 'Beneficiary' or\
               key == 'Topic':
                for theme_word in value.split():
                    if theme_word == 'any':
                        quantifier = 'pi'
                        value = value[4:]
                    if theme_word == 'a':
                        number = 1
                        quantifier = 'si'
                        value = value[2:]
                    if theme_word == 'the':
                        number = 1
                        quantifier = 'sd'
                        value = value[4:]
                        
                    if theme_word == '*' or theme_word == 'you':
                        obj = 'Junior'
                    elif theme_word == 'I' or theme_word == 'me':
                        obj = 'Commander'
                    else:
                        # If there is a number, split it from the theme
                        if theme_word.isdigit():
                            number = word
                            obj = lemmatizer.lemmatize(obj_string_list[-1], 'n')
                        else:
                            obj = lemmatizer.lemmatize(value,'n')
            if key == 'Location' or key == 'Destination':
                location = value
            if key == 'VERB':
                verb = lemmatizer.lemmatize(value, pos='v').lower()

        env_query = EnvironmentQuery(quantifier, obj, location)
        command = None

        if is_question(str(frame[1])):
            semantic_representation_list.append(env_query)
        else:
            if verb is not None and verb != 'be':
                command = Command(verb, env_query)

            if not command is None:
                semantic_representation_list.append(command)
            else:
                env_statement = EnvironmentStatement(number, 'exact', obj, location)
                semantic_representation_list.append(env_statement)

    return semantic_representation_list
                

if __name__ == '__main__':
    # Take stdin as input
    #parse_tree_string = """(S  (NP-SBJ-A (-NONE- *)) (VP (VB Defuse) (NP-A  (NP (DT any) (NNS bombs)) (SBAR (IN that) (S-A  (NP-SBJ-A (PRP you)) (VP  (VP-A (VBP see))(CC and) (VP-A (VBP notify) (NP-A (PRP me))))))))(. .))"""
    #parse_tree_string = """(S (CC And) (SBAR-ADV (IN if) (S-A  (NP-SBJ-A-0 (PRP you)) (VP (VBP get) (VP-A (VBN flipped) (NP-A-0 (-NONE- *))))))(, ,) (NP-SBJ-A (-NONE- *)) (VP (VB call) (NP-A (PRP me)))(. .))"""
    #parse_tree_string = """(S  (NP-SBJ-A (-NONE- *)) (VP (VB Tell) (NP-A (PRP me)) (SBAR-ADV (IN if) (S-A  (NP-SBJ-A (PRP you)) (VP (VBP see) (NP-A  (NP (DT a) (JJ bad) (NN guy))(CC or) (NP (DT a) (NN hostage)))))))(. .))"""
    #parse_tree_string = """(S (NP-SBJ-A (-NONE- *)) (VP (VB Tell) (NP-A (PRP me))) (. .))"""
    #parse_tree_string = """((S (NP-SBJ-A (-NONE- *)) (VP(VB Tell) (NP-A (PRP me)) (SBAR-A(WHNP-0 (WDT what)) (S-A(NP-SBJ-0 (-NONE- *T*)) (VP (VBZ 's) (PP-LOC-PRD (IN in) (NP-A (NN room) (CD 3)))))))                                                 (. ?)))"""
    parse_tree_string = """((S (NP-SBJ-A (-NONE- *)) (VP (VP-A (VB Go) (PP-CLR (TO to) (NP-A (NN room) (CD 3)))) (CC and) (VP-A (VB defuse) (NP-A (DT the) (NN bomb)))) (. .)))"""
    #parse_tree_string = """((S (NP-SBJ-A (-NONE- *)) (VP (VB Go) (PP-CLR (TO to) (NP-A (NN room) (CD 3)))) (. .))) """
    parse_tree_string = """((SINV (VP (VBP Are) (ADVP-LOC-PRD (RB there))) (NP-SBJ (NP (DT any) (NNS hostages)) (PP-LOC (IN in) (NP-A (NN room) (CD 1)))) (. ?)))"""
    parse_tree_string = """(S (NP-SBJ-A (EX There)) (VP (VBP are) (NP-PRD-A (NP (CD 2) (NNS hostages)) (PP-LOC (IN in) (NP-A (NN room) (CD 3))))) (. .))"""
    parse_tree_string = """((SBARQ (WHADVP-0 (WRB Where)) (SQ (VP (VBP are) (NP-PRD-A (DT the) (NNS hostages)) (ADVP-0 (-NONE- *T*)))) (. ?)))"""
    
    result = extract_frames_from_parse(parse_tree_string)
    print result
    
    semantic_structures = create_semantic_structures(result)

    for structure in semantic_structures:
        print structure


##    if is_question(parse_tree_string):
##        print get_question_semantics_from_parse_tree(parse_tree_string)
##    else:
##        print get_stupid_semantics_from_parse_tree(parse_tree_string)
            

            

