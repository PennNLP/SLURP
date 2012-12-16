"""
Takes a parse tree string and creates semantic structures to be read by Knowledge.
"""

# Copyright (C) 2011=2012 Ian Perera and Constantine Lignos
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

import re
import frames
import string
from tree import Tree
from wntools import morphy
from collections import defaultdict
from new_structures import *
from lexical_constants import *

def get_semantics_from_parse_tree(parse_tree_string):
    """Take a string representing the parse tree as input, and print the
    semantic parse. The result list consists of a list of tuples, with each
    tuple containing the VerbNet frame and its associated tree."""
    result_list = []
    
    # In case we're handed an bad string, bail somewhat gracefully
    try:
        parse_tree = Tree.parse(parse_tree_string)
    except ValueError:
        print "Warning: semantics could not parse tree", repr(parse_tree_string)
        return result_list

    # Split clauses to handle them separately
    split_clause_dict = frames.split_clauses(parse_tree)

    # Activize clauses
    for key, (clause, conjunction) in split_clause_dict.items():
        activized_clause = frames.activize_clause(clause)
        split_clause_dict[key] = (activized_clause, conjunction)
        
    for (clause, conjunction) in split_clause_dict.values():
        # Split conjunctions and duplicate arguments if necessary
        split_tree_dict = frames.split_conjunctions(clause)
        
        if conjunction != '':
            result_list.append(conjunction)
        
        for (split_tree, conjunction) in split_tree_dict.values():
            if conjunction != '':
                result_list.append(conjunction)

            for tree in split_tree:
                tag_list = []

                # Store whether there was an existential there
                if frames.is_existential(str(tree)):
                    tag_list.append('ex')

                # Transformational grammar stuff
                tree = frames.existential_there_insertion(tree)
                print 'Before inversion:'
                print str(tree)
                tree = frames.invert_clause(tree)
                print 'After inversion:'
                print str(tree)
                tree = frames.wh_movement(tree)
                tree = frames.negation_inversion(tree)

                verbs, tree = frames.find_verbs(tree)
                
                # Create VFOs for each verb, then match them to the parse tree
                for verb,negation in verbs:
                    lemmatized_verb = morphy(verb,'v')
                    vfo_list = frames.create_VerbFrameObjects(lemmatized_verb)
                    match_list = []
                    
                    print 'VFO list:'
                    print '\n'.join(str(vfo.frame_list) for vfo in vfo_list)

                    for vfo in vfo_list:
                        match = vfo.match_parse(tree)
                        
                        if match:
                            print 'Matched:'
                            print '\t',str(vfo.frame_list)
                            print 'with'
                            print '\t',str(tree)
                            match_list.append((match, vfo.classid))
                    
                    print 'Match list:'
                    for m in match_list:
                        for a,b in m[0].items():
                            print a,str(b)
                        print '\n\n'
                        
                    (best_match, sense) = frames.pick_best_match(match_list)
                    if not best_match is None:
                        result_list.append((best_match, tree, tag_list, sense, verb, negation))
                    
    return result_list
    

def extract_entity(parse_tree, semantic_role = ''):
    """Creates an entity object given a snippet of a parse tree."""
    entity = Location() if semantic_role == 'Location' else Object()
    '''
    print 'Extracting from:'
    print str(parse_tree)
    '''
    # Ignore rescursed trees and added descriptions
    ignore_positions = []
    for position in parse_tree.treepositions():
        if not isinstance(parse_tree[position], Tree):
            continue
        if position in ignore_positions:
            continue
        subtree = parse_tree[position]
        node = subtree.node
        leaves = ' '.join(subtree.leaves()).lower()
        # A noun phrase might have sub-parts that we need to parse recursively
        # Recurse while there are NP's below the current node
        if subtree is not parse_tree and 'NP' in node:
            entity.merge(extract_entity(subtree))
            # Ignore positions should be relative to parse_tree
            ignore_positions.extend(position + subposition for subposition in subtree.treepositions())
        # A determiner cardinal node adds some information for the quantifier
        if 'DT' in node:
            entity.quantifier.fill_determiner(leaves)
        # Cardinal number sets the quantifier number
        elif node == 'CD':
            entity.quantifier.fill_cardinal(leaves)
        
        elif node == 'PRP':
            entity.name = 'Commander' if leaves in ('i','me') else leaves
        elif 'NN' in node or node == '-NONE-':
            entity.name = morphy(leaves, 'n')
            if entity.name is None:
                entity.name = leaves

        elif 'PP' in node or node in ('SBAR','JJ'):
            entity.description.append(leaves)
            # Ignore positions should be relative to parse_tree
            ignore_positions.extend(position + subposition for subposition in subtree.treepositions())
    return entity


def create_semantic_structures(frame_semantic_list):
    """Take in the list of VerbNet frames and generate the semantic
    representation structures from them."""
    semantic_representation_list = []
    conditional = False
    # Frame semantic list is a list of conjunction strings and tuples, where the
    # first element of the tuple is the frame semantics, the second element is
    # the original tree branch it came from, the third is a list of tags, and
    # the fourth is the sense.

    for frame in frame_semantic_list:
        # Check that this is a VerbNet frame and not a conjunction
        try:
            frame_items = frame[0].items()
        except AttributeError:
            if 'if' in str(frame):
                conditional = True
            frame_items = None
            semantic_representation_list.append(frame)
            continue
        
        sense = frame[3].split('-')[0]
        # Get the action associated with the sense
        # If such a mapping does not exist, use the original verb
        action = ACTION_ALIASES.get(sense, frame[4])

        item_to_entity = {key:extract_entity(value, key) for key,value in frame_items}

        wh_question_type = frames.get_wh_question_type(str(frame[1]))
        
        # If it's a WH-question, find the type of question it is and add the object
        if wh_question_type is not None and 'Theme' in item_to_entity:
            if wh_question_type == 'Location':
                semantic_representation_list.append(NewLocationQuery(item_to_entity['Theme']))
            elif wh_question_type == 'Status':
                semantic_representation_list.append(NewStatusQuery(item_to_entity['Theme']))
            elif wh_question_type in ('People','Entity'):
                semantic_representation_list.append(NewEntityQuery(item_to_entity['Location']))
                
        # If it's a yes-no question, add the theme and location of the question
        elif frames.is_yn_question(str(frame[1])):
            if 'Theme' in item_to_entity and 'Location' in item_to_entity:
                semantic_representation_list.append(NewYNQuery(item_to_entity['Theme'],item_to_entity['Location']))
        # If it's a conditional statement, the first statement is an event
        elif conditional is True and 'Theme' in item_to_entity:
            semantic_representation_list.append(NewEvent(item_to_entity['Theme'], action))
        # It's a regular command
        elif action is not None and action not in  ('is', 'are', 'be'):
            theme = item_to_entity.get('Theme',None)
            agent = item_to_entity.get('Agent',None)
            patient = item_to_entity.get('Patient',None)
            if patient is None:
                patient = item_to_entity.get('Recipient',None)
            location = item_to_entity.get('Location',None)
            semantic_representation_list.append(NewCommand(agent,theme,patient,location,action,negation=frame[5]))
        # It's an assertion
        else:
            semantic_representation_list.append(NewAssertion(item_to_entity.get('Theme',None), \
                                                          item_to_entity.get('Location',None),\
                                                          'ex' in frame[2]))

    return semantic_representation_list
