"""
Takes a parse tree string and creates semantic structures to be read by Knowledge.
"""

# Copyright (C) 2011-2013 Ian Perera, Constantine Lignos, and Kenton Lee
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

from semantics.frames import split_clauses, activize_clause, is_existential, invert_clause, \
    get_wh_question_type, is_yn_question, pick_best_match, existential_there_insertion, \
    wh_movement, split_conjunctions, find_verbs, create_VerbFrameObjects
from semantics.tree import Tree
from semantics.wntools import morphy
from semantics.new_structures import Location, ObjectEntity, YNQuery, StatusQuery, \
    EntityQuery, Command, LocationQuery, Assertion, Event
from semantics.lexical_constants import ACTION_ALIASES


EXTRACT_DEBUG = False


def extract_frames_from_parse(parse_tree_string, verbose=False):
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

    # Temporarily (and maybe permanently) disabled features:
    # 1. Clause splitting: we have not found any example where it does something
    # 2. Activizing clauses: for now, passives do not matter.

    # Split clauses to handle them separately
    #split_clause_dict = split_clauses(parse_tree)

    # Activize clauses
    #for key, (clause, conjunction) in split_clause_dict.items():
    #    activized_clause = activize_clause(clause)
    #    split_clause_dict[key] = (activized_clause, conjunction)

    #for (clause, conjunction) in split_clause_dict.values():
    for clause, conjunction in ((parse_tree, ''),):
        # Split conjunctions and duplicate arguments if necessary
        split_tree_dict = split_conjunctions(clause)

        if conjunction != '':
            result_list.append(conjunction)

        for (split_tree, conjunction) in split_tree_dict.values():
            if conjunction != '':
                result_list.append(conjunction)

            for tree in split_tree:
                tag_list = []

                # Store whether there was an existential there
                if is_existential(str(tree)):
                    tag_list.append('ex')

                # Transformational grammar stuff
                tree = existential_there_insertion(tree)
                tree = invert_clause(tree)
                tree = wh_movement(tree)

                if EXTRACT_DEBUG:
                    print 'Transformed tree:'
                    print str(tree)

                verbs = find_verbs(tree)

                # Create VFOs for each verb, then match them to the parse tree
                for verb, negation in verbs:
                    lemmatized_verb = morphy(verb, 'v')
                    vfo_list = create_VerbFrameObjects(lemmatized_verb)
                    match_list = []

                    if EXTRACT_DEBUG:
                        print 'VFO list for %s:' % verb
                        print '\n'.join(str(vfo.frame_list) for vfo in vfo_list)

                    for vfo in vfo_list:
                        match = vfo.match_parse(tree)

                        if match:
                            if EXTRACT_DEBUG:
                                print 'Matched:'
                                print '\t', str(vfo.frame_list)
                                print 'with'
                                print '\t', str(tree)
                            match_list.append((match, vfo.classid))

                    if EXTRACT_DEBUG:
                        print 'Match list:'

                        for m in match_list:
                            print 'Sense:', m[1]
                            for a, b in m[0].items():
                                print a, str(b)
                            print '\n\n'

                    (best_match, sense) = pick_best_match(match_list)

                    if EXTRACT_DEBUG:
                        print 'Chose: '
                        if best_match:
                            print sense
                            for a, b in best_match.items():
                                print a, str(b)
                        else:
                            print str(None)
                        print '\n\n'
                    if not best_match is None:
                        result_list.append((best_match, tree, tag_list, sense, verb, negation))

    return result_list


def extract_entity(parse_tree, semantic_role=''):
    """Creates an entity object given a snippet of a parse tree."""
    entity = Location() if semantic_role in ('Location', 'Source', 'Destination') else ObjectEntity()

    # print 'Extracting from:'
    # print str(parse_tree)

    # Ignore rescursed trees and added descriptions
    ignore_positions = []
    previous_node = None
    previous_leaves = None
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
            # ignore_positions should be relative to parse_tree
            ignore_positions.extend(position + subposition for subposition in subtree.treepositions())
        # A determiner cardinal node adds some information for the quantifier
        if 'DT' in node:
            entity.quantifier.fill_determiner(leaves)
        # Cardinal number sets the quantifier number
        elif node == 'CD':
            entity.quantifier.fill_cardinal(leaves)
            if entity.quantifier.number is None:
                # Not actually a number
                entity.name = leaves
        elif node == 'PRP':
            entity.name = 'Commander' if leaves in ('i', 'me') else leaves
        elif ('PP' in node and entity.name) or node in ('SBAR', 'JJ'):
            entity.description.append(leaves)
            # ignore_positions should be relative to parse_tree
            ignore_positions.extend(position + subposition for subposition in subtree.treepositions())
        elif 'NN' in node and previous_node and 'NN' in previous_node and entity.name == previous_leaves:
            entity.description.append(previous_leaves)
            entity.name = leaves
        elif 'NN' in node or node == '-NONE-':
            entity.name = morphy(leaves, 'n')
            if entity.name is None:
                entity.name = leaves
        elif node == 'RB' and leaves == 'there':
            entity.name = 'there'
        previous_node = node
        previous_leaves = leaves
    return entity


def create_semantic_structures(frame_semantic_list):
    """Take in the list of VerbNet frames and generate the semantic
    representation structures from them."""
    semantic_representation_list = []
    isConditionalNext = False
    # Whether there is a conditional in the semantic representation list waiting to be picked up
    hasConditional = False

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
                isConditionalNext = True
            frame_items = None
            #semantic_representation_list.append(frame)
            continue

        sense = frame[3].split('-')[0]
        # Get the action associated with the sense
        # If such a mapping does not exist, use the original verb
        action = ACTION_ALIASES.get(sense, frame[4])

        item_to_entity = {key: extract_entity(value, key) for key, value in frame_items}

        wh_question_type = get_wh_question_type(str(frame[1]))

        # If it's a WH-question, find the type of question it is and add the object
        if wh_question_type is not None:
            if wh_question_type == 'Status':
                semantic_representation_list.append(StatusQuery())
            elif 'Theme' in item_to_entity:
                if wh_question_type == 'Location':
                    semantic_representation_list.append(LocationQuery(item_to_entity['Theme']))
                elif wh_question_type in ('People', 'Entity'):
                    semantic_representation_list.append(EntityQuery(item_to_entity['Location']))

        # If it's a yes-no question, add the theme and location of the question
        elif is_yn_question(str(frame[1])):
            if 'Theme' in item_to_entity and 'Location' in item_to_entity:
                semantic_representation_list.append(YNQuery(item_to_entity['Theme'], item_to_entity['Location']))
        # If it's a conditional statement, it is modifying
        # either the previous or next structure
        elif isConditionalNext:
            if 'Stimulus' in item_to_entity:
                condition = Event(item_to_entity['Stimulus'], action)
            else:
                condition = Assertion(item_to_entity.get('Theme', None),
                                      item_to_entity.get('Location', None),
                                      'ex' in frame[2])

            if len(semantic_representation_list) > 0 and isinstance(semantic_representation_list[-1], Command):
                # Found the command to which this condition belongs
                semantic_representation_list[-1].condition = condition
            else:
                # Save it for later
                semantic_representation_list.append(condition)
                hasConditional = True
            # Consume the conditional
            isConditionalNext = False
        # It's a regular command
        elif action is not None and action not in ('is', 'are', 'be'):
            theme = item_to_entity.get('Theme', None)
            agent = item_to_entity.get('Agent', None)
            patient = item_to_entity.get('Patient', None)
            if patient is None:
                patient = item_to_entity.get('Recipient', None)
            location = item_to_entity.get('Location', None)
            source = item_to_entity.get('Source', None)
            destination = item_to_entity.get('Destination', None)
            current_command = Command(agent, theme, patient, location, source, destination, action, negation=frame[5])
            # Try to pick up the previous condition
            if hasConditional:
                current_command.condition = semantic_representation_list.pop()
                hasConditional = False
            semantic_representation_list.append(current_command)
        # It's an assertion
        else:
            semantic_representation_list.append(Assertion(item_to_entity.get('Theme', None),
                                                          item_to_entity.get('Location', None),
                                                          'ex' in frame[2]))
    if EXTRACT_DEBUG:
        print 'Semantic representation list:'
        print semantic_representation_list
    return semantic_representation_list


def process_parse_tree(parse_tree_input, text_input, knowledge_base=None, quiet=False):
    """Produces semantic interpretations of parse trees."""
    if not quiet:
        print "Processing:", repr(text_input)

    # Perform tree operations
    frames = extract_frames_from_parse(parse_tree_input)
    if not quiet:
        print "Semantic frames:", frames

    # Extract meaning
    semantic_structures = create_semantic_structures(frames)

    # Update KB
    if knowledge_base:
        kb_response = knowledge_base.process_semantic_structures(semantic_structures, source='cmdr')
    else:
        kb_response = ''

    # Extract commands.
    new_commands = [item for item in semantic_structures if isinstance(item, Command)]
    if knowledge_base:
        knowledge_base.fill_commands(new_commands)
    if new_commands and not quiet:
        print "New commands:"
        for command in new_commands:
            print command

    if kb_response and not quiet:
        print "KB response:", kb_response

    return (frames, new_commands, kb_response)
