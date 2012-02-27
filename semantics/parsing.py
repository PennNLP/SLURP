"""Ian Perera

Takes a parse tree string and creates semantic structures to be read by
Knowledge."""

# -*- coding: iso-8859-1 -*-
import re
import Frames
import string
from Tree import Tree
from wntools import morphy
from collections import defaultdict
from Structures import Predicate, Quantifier, EntityClass, \
                                     Command, Assertion, YNQuery, WhQuery, Event


POSSIBLE_OBJECTS = ['hostage', 'bomb', 'badguy', 'terrorist', 'room']


def get_semantics_from_parse_tree(parse_tree_string):
    """Take a string representing the parse tree as input, and print the
    semantic parse. The result list consists of a list of tuples, with each
    tuple containing the VerbNet frame and its associated tree."""
    parse_tree = Tree.parse(parse_tree_string)

    # Split clauses to handle them separately
    split_clause_dict = Frames.split_clauses(parse_tree)

    # Activize clauses
    for key, (clause, conjunction) in split_clause_dict.items():
        activized_clause = Frames.activize_clause(clause)
        split_clause_dict[key] = (activized_clause, conjunction)

    result_list = []
        
    for (clause, conjunction) in split_clause_dict.values():
        # Split conjunctions and duplicate arguments if necessary
        split_tree_dict = Frames.split_conjunctions(clause)
        
        if conjunction != '':
            result_list.append(conjunction)
        
        for (split_tree, conjunction) in split_tree_dict.values():
            if conjunction != '':
                result_list.append(conjunction)

            for tree in split_tree:
                tag_list = []

                # Store whether there was an existential there
                if Frames.is_existential(str(tree)):
                    tag_list.append('ex')

                # Transformational grammar stuff
                tree = Frames.existential_there_insertion(tree)
                tree = Frames.invert_clause(tree)
                tree = Frames.wh_movement(tree)



                # Regex for finding verbs 
                verb_finder = re.compile(r'(?<=VB[ DGNPZ]) *\w*(?=\))')

                # Get the lemma of the verb for searching verbnet
                verbs = (word.strip().lower() for word in
                         verb_finder.findall(str(tree)))

                # Create VFOs for each verb, then match them to the parse tree
                for verb in verbs:
                    lemmatized_verb = morphy(verb,'v')
                    vfo_list = Frames.create_VerbFrameObjects(lemmatized_verb)

                    match_list = []
                    
                    for vfo in vfo_list:
                        match = vfo.match_parse(tree)
                        
                        if match:
                            match_list.append((match, vfo.classid))

                    (best_match, sense) = Frames.pick_best_match(match_list)
                    if not best_match is None:
                        result_list.append((best_match, tree, tag_list, sense))
                    

    return result_list

def text2int(textnum):
    """From recursive at
    http://stackoverflow.com/questions/493174/is-there-a-way-to-convert-number-words-to-integers-python

    Converts number words to integers.
    """
    numwords = {}
    units = [
    "zero", "one", "two", "three", "four", "five", "six", "seven", "eight",
    "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen",
    "sixteen", "seventeen", "eighteen", "nineteen",
    ]

    tens = ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"]

    scales = ["hundred", "thousand", "million", "billion", "trillion"]

    numwords["and"] = (1, 0)
    for idx, word in enumerate(units):
        numwords[word] = (1, idx)
    for idx, word in enumerate(tens):
        numwords[word] = (1, idx * 10)
    for idx, word in enumerate(scales):
        numwords[word] = (10 ** (idx * 3 or 2), 0)

    current = result = 0
    for word in textnum.split():
        if word not in numwords:
            raise Exception("Illegal word: " + word)

        scale, increment = numwords[word]
        current = current * scale + increment
        if scale > 100:
            result += current
            current = 0

    return result + current

def extract_entity_class(parse_tree, semantic_role = ''):
    """Creates an entity_class object given a snippet of a parse tree."""
    quantifier = Quantifier()
    quantifier.number = 1
    quantifier.definite = True

    predicates = defaultdict(list)

    for position in parse_tree.treepositions():
        if not isinstance(parse_tree[position], Tree):
            continue
        
        subtree = parse_tree[position]
        node = subtree.node

        # A determiner node adds some information for the quantifier
        if node == 'DT':
            determiner = ' '.join(subtree.leaves()).lower()
            if determiner == 'any':
                quantifier.definite = False
                quantifier.exhaustive = True
                quantifier.proportionality = 'at least'
                quantifier.number = 1
            if determiner == 'a' or determiner == 'an':
                quantifier.plural = False
                quantifier.number = 1
                quantifier.definite = False
                quantifier.exhaustive = False
                quantifier.proportionality = 'at least'
            if determiner == 'the':
                quantifier.definite = True
                quantifier.exhaustive = False
        # A personal pronoun adds some information for the quantifier
        elif node == 'PRP':
            pronoun = ' '.join(subtree.leaves()).lower()
            if pronoun == 'him' or pronoun == 'he' or \
               pronoun == 'her' or pronoun == 'she':
                obj = pronoun
                quantifier.definite = True
                quantifier.number = 1
                quantifier.proportionality = 'exact'
                quantifier.exhaustive = True
                quantifier.fulfilled = False
            # The object is Commander
            elif pronoun == 'i' or pronoun == 'me':
                obj = 'Commander'
                quantifier.definite = True
                quantifier.number = 1
                quantifier.proportionality = 'exact'
                quantifier.exhaustive = True
            else:
                obj = pronoun
                quantifier.definite = True
                quantifier.number = 1
                quantifier.proportionality = 'exact'
                quantifier.exhaustive = True
                quantifier.fulfilled = False

            predicates[semantic_role].append(Predicate(semantic_role,obj))
        # Prepositional phrase generates a location predicate
        elif node == 'PP-LOC':
            for subposition in subtree.treepositions():
                predicate_type = 'Location'
                if not isinstance(subtree[subposition], Tree):
                    continue
                if subtree[subposition].node == 'IN':
                    predicate_type = 'Location'
                elif 'NP' in subtree[subposition].node and \
                     'NP' not in subtree[subposition][0]:
                    predicates[predicate_type].append(
                        Predicate(predicate_type,
                                  ' '.join(
                                      subtree[subposition].leaves())))
        # Cardinal number sets the quantifier number
        elif node == 'CD':
            number_text = ' '.join(subtree.leaves()).lower()
            if not number_text.isdigit():
                number = text2int(number_text)
                quantifier.number = number

        # A noun phrase might have sub-parts that we need to parse separately
        elif ('NP' in node) or node == 'NP-PRD-A':
            obj_word_list = []
            for subposition in subtree.treepositions():
                # Don't check leaves or parents of leaves
                if (not isinstance(subtree[subposition], Tree) or
                        isinstance(subtree[subposition][0], Tree)):
                    continue

                theme_word = ' '.join(subtree[subposition].leaves()).lower()
                if theme_word is None:
                    continue

                # Get the actual object in question
                if ('NN' in subtree[subposition].node or 'CD' in subtree[subposition].node or 
                    'JJ' in subtree[subposition].node and theme_word not in obj_word_list):
                    m_word = morphy(theme_word, 'n')
                    if m_word is None:
                        m_word = theme_word
                    obj_word_list.append(m_word)

                # Get the quantifier info
                if len(obj_word_list) > 0 and quantifier.plural is not None:
                    if obj_word_list[0] != theme_word:
                        quantifier.proportionality = 'at least'
                        quantifier.number = 1
                    else:
                        quantifier.proportionality = 'exact'
                        quantifier.number = 1

            # Compile object reference into lower_case_with_underscores name
            if len(obj_word_list) > 0:
                obj_name = "_".join(word.lower() for word in obj_word_list)
                predicates[semantic_role].append(Predicate(semantic_role, obj_name))
                break

        # If it's just a noun, add it as a predicate
        elif 'N' in node and 'SBJ' not in node:
            predicates[semantic_role].append(Predicate(semantic_role,' '.join(subtree.leaves())))
            quantifier.definite = True
        elif 'ADV' in node:
            predicates[semantic_role].append(Predicate(semantic_role,' '.join(subtree.leaves())))
            
    entity_class = EntityClass(quantifier,predicates)

    return entity_class


def create_semantic_structures(frame_semantic_list):
    """Take in the list of VerbNet frames and generate the semantic
    representation structures from them."""
    semantic_representation_list = []
    conditional = False
    # Frame semantic list is a list of conjunction strings and tuples, where the
    # first element of the tuple is the frame semantics, the second element is
    # the original tree branch it came from, and the third is a list of tags.

    for frame in frame_semantic_list:
        # Check that this is a VerbNet frame and not a conjunction
        try:
            frame_items = frame[0].items()
        except:
            if 'if' in str(frame):
                conditional = True
            frame_items = None
            semantic_representation_list.append(frame)
            continue

        verb = frame[3].split('-')[0]

        location_predicates = defaultdict(list)
        location_resolved = False

        entity_class_dict = {}

        for key, value in frame_items:
            entity_class_dict[key] = extract_entity_class(value, key)        
        
        wh_question_type = Frames.get_wh_question_type(str(frame[1]))
        
        # If it's a WH-question, find the type of question it is and add the object
        if wh_question_type is not None and 'Theme' in entity_class_dict:
            semantic_representation_list.append(\
                WhQuery(entity_class_dict['Theme'], wh_question_type))
        # If it's a yes-no question, add the theme of the question
        elif Frames.is_yn_question(str(frame[1])):
            if 'Theme' in entity_class_dict and 'Location' in entity_class_dict:
                entity_class_dict['Theme'].predicates = \
                                        dict(entity_class_dict['Theme'].predicates.items() +
                                                entity_class_dict['Location'].predicates.items() )
                semantic_representation_list.append(\
                    YNQuery(entity_class_dict['Theme']))
        # If there is a specific location involved, it's a command (this may need to be changed)
        elif location_resolved is True:
            semantic_representation_list.append(
                Command(str(location_predicates['Location'][0]),'go'))
            # If there was a conditional statement, this command ends it
            conditional = False
        # If it's a conditional statement, the first statement is an event
        elif conditional is True and 'Theme' in entity_class_dict:
            semantic_representation_list.append(Event(entity_class_dict['Theme'], verb))
        # It's a regular command
        elif verb is not None and verb != 'be':
            command_predicate_dict = {}
            for key, value in entity_class_dict.items():  
                # Only add semantic roles, not prepositions and verbs
                if key == string.capitalize(key):
                    command_predicate_dict[key] = value.predicates[key]
            
            for key, value in entity_class_dict.items():
                if key == 'Theme' or key == 'Agent' or key == 'Patient':
                    semantic_representation_list.append(Command(
                                                            EntityClass(
                                                                value.quantifier,
                                                                command_predicate_dict),
                                                            verb))
                    break
        # It's an assertion
        else:
            semantic_representation_list.append(\
                Assertion(entity_class_dict['Theme'], entity_class_dict['Location'].predicates,\
                          ('ex' in frame[2])))

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
    
    result = get_semantics_from_parse_tree(parse_tree_string)
    print result
    
    semantic_structures = create_semantic_structures(result)

    for structure in semantic_structures:
        print structure


##    if is_question(parse_tree_string):
##        print get_question_semantics_from_parse_tree(parse_tree_string)
##    else:
##        print get_stupid_semantics_from_parse_tree(parse_tree_string)
            

            

