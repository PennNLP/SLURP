"""
Takes a parse tree string and creates semantic structures to be read by Knowledge.
"""

# Copyright (C) 2011-2013 Eric Doty, Constantine Lignos, Kenton Lee, and Ian Perera
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

# TODO: Refactor to remove all REs
import re
from copy import deepcopy

from semantics.frames import (best_matching_frame, get_verb_frames)
from semantics.tree import Tree
from semantics.wntools import morphy
from semantics.new_structures import (
    Location, ObjectEntity, YNQuery, StatusQuery,
    EntityQuery, Command, LocationQuery, Assertion, Event)
from semantics.lexical_constants import ACTION_ALIASES

EXTRACT_DEBUG = False
MATCH_DEBUG = True
STRICT_MODE = True
ALLOW_LEFTOVERS = False

# Parse tree constants
DASH = "-"
A_TAG = "A"
VP_TAG = "VP"
S_TAG = "S"
SUBJ_TAG = "NP-SBJ"
VERB_TAG = "VB"
ADVP_TAG = "ADVP"
DO_WORD = "do"
SUPPORT_VERBS = set((DO_WORD, "please"))
NOT_WORDS = set(("not", "n't"))
NEVER_WORD = "never"
CONDITIONAL_MARKERS = set(['if', 'when'])
AUXILIARY_VERBS = set(['was', 'is', 'get', 'are', 'got', 'were', 'been', 'being'])


def split_tag(tag):
    """Split a tag into (tag, dash_tag), excluding tags like -A.
    >>> split_tag('NP')
    ('NP', None)
    >>> split_tag('NP-SBJ')
    ('NP', 'SBJ')
    >>> split_tag('NP-SBJ-A')
    ('NP', 'SBJ')
    >>> split_tag('NP-A')
    ('NP', None)

    """
    pieces = tag.split(DASH)
    tag = pieces[0]
    dash_tag = pieces[1] if (len(pieces) > 1 and pieces[1] != A_TAG) else None
    return (tag, dash_tag)


def main_tag(tag):
    """Return the main tag, the core tag with no dashtags.
    >>> main_tag('NP')
    'NP'
    >>> main_tag('NP')
    'NP'
    >>> main_tag('NP-SBJ-A')
    'NP'
    >>> main_tag('NP-A')
    'NP'

    """
    return split_tag(tag)[0]


class FrameMatch(object):
    """Represents a successful match of a VerbFrame to a subtree."""

    def __init__(self, verb, sense, argdict, negated, tree):
        self.verb = verb
        self.sense = sense
        if not argdict:
            raise ValueError("Cannot create a FrameMatch with empty arguments")
        self.args = argdict
        self.negated = negated
        self.tree = tree
        self.condition = None
        self.condition_head = None

    def set_conditional(self, condition_head, condition):
        """Make this frame conditional on a head and VerbFrame condition."""
        self.condition_head = condition_head
        self.condition = condition

    def is_conditional(self):
        """Return whether this frame is conditioned on another frame."""
        return self.condition is None

    def is_negated(self):
        """Return whether this frame is negated."""
        return self.negated

    def __str__(self):
        return ("<FrameMatch verb: {} sense: {} args: {} negated: {} condition: {}>".format(
            self.verb, self.sense,
            ["{}: {}".format(arg, " ".join(tree.leaves())) for arg, tree in self.args.items()],
            self.negated, " ".join([str(self.condition_head), str(self.condition)])))

    def __repr__(self):
        return str(self)


def extract_frames_from_parse(parse_tree_string, verbose=False):
    """Take a string representing the parse tree as input, and print the
    semantic parse. The result list consists of a list of tuples, with each
    tuple containing the VerbNet frame and its associated tree."""
    # TODO: Use verbose argument
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
    # split_clause_dict = split_clauses(parse_tree)

    # Activize clauses
    # for key, (clause, conjunction) in split_clause_dict.items():
    #     activized_clause = activize_clause(clause)
    #     split_clause_dict[key] = (activized_clause, conjunction)

    # for (clause, conjunction) in split_clause_dict.values():
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

                # TODO: Update to new match_verbs
                verbs = match_verbs(tree)

                # Create VFOs for each verb, then match them to the parse tree
                for verb, negation, subtree in verbs:
                    vfo_list = get_verb_frames(verb)
                    match_list = []

                    if EXTRACT_DEBUG:
                        print 'VFO list for %s:' % verb
                        print '\n'.join(str(vfo.frame_list) for vfo in vfo_list)

                    for vfo in vfo_list:
                        match = vfo.match_parse(subtree)

                        if match:
                            if EXTRACT_DEBUG:
                                print 'Matched:'
                                print str(vfo.frame_list)
                                print str(tree)
                                print
                            match_list.append((match, vfo.classid))

                    if EXTRACT_DEBUG:
                        print 'Match list:'
                        for m in match_list:
                            print 'Sense:', m[1]
                            for a, b in m[0].items():
                                print a, str(b)
                            print
                        print

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
                        result_list.append(
                            (best_match, tree, tag_list, sense, verb, negation))

    return result_list


def extract_entity(parse_tree, semantic_role=''):
    """Creates an entity object given a snippet of a parse tree."""
    entity = Location() if semantic_role in (
        'Location', 'Source', 'Destination') else ObjectEntity()

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
            ignore_positions.extend(
                position + subposition for subposition in subtree.treepositions())
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
            ignore_positions.extend(
                position + subposition for subposition in subtree.treepositions())
        elif ('NN' in node and previous_node and
              'NN' in previous_node and entity.name == previous_leaves):
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
    # Whether there is a conditional in the semantic representation list
    # waiting to be picked up
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
            # semantic_representation_list.append(frame)
            continue

        sense = frame[3].split('-')[0]
        # Get the action associated with the sense
        # If such a mapping does not exist, use the original verb
        action = ACTION_ALIASES.get(sense, frame[4])

        item_to_entity = {key: extract_entity(value, key)
                          for key, value in frame_items}

        wh_question_type = get_wh_question_type(str(frame[1]))

        # If it's a WH-question, find the type of question it is and add the
        # object
        if wh_question_type is not None:
            if wh_question_type == 'Status':
                semantic_representation_list.append(StatusQuery())
            elif 'Theme' in item_to_entity:
                if wh_question_type == 'Location':
                    semantic_representation_list.append(
                        LocationQuery(item_to_entity['Theme']))
                elif wh_question_type in ('People', 'Entity'):
                    semantic_representation_list.append(
                        EntityQuery(item_to_entity['Location']))

        # If it's a yes-no question, add the theme and location of the question
        elif is_yn_question(str(frame[1])):
            if 'Theme' in item_to_entity and 'Location' in item_to_entity:
                semantic_representation_list.append(
                    YNQuery(item_to_entity['Theme'], item_to_entity['Location']))
        # If it's a conditional statement, it is modifying
        # either the previous or next structure
        elif isConditionalNext:
            if 'Stimulus' in item_to_entity:
                condition = Event(item_to_entity['Stimulus'], action)
            else:
                condition = Assertion(item_to_entity.get('Theme', None),
                                      item_to_entity.get('Location', None),
                                      'ex' in frame[2])

            if (len(semantic_representation_list) > 0 and
               isinstance(semantic_representation_list[-1], Command)):
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
            current_command = Command(
                agent, theme, patient, location, source, destination, action, negation=frame[5])
            # Try to pick up the previous condition
            if hasConditional:
                current_command.condition = semantic_representation_list.pop()
                hasConditional = False
            semantic_representation_list.append(current_command)
        # It's an assertion
        else:
            semantic_representation_list.append(
                Assertion(item_to_entity.get('Theme', None),
                          item_to_entity.get('Location', None), 'ex' in frame[2]))
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
        kb_response = knowledge_base.process_semantic_structures(
            semantic_structures, source='cmdr')
    else:
        kb_response = ''

    # Extract commands.
    new_commands = [
        item for item in semantic_structures if isinstance(item, Command)]
    if knowledge_base:
        knowledge_base.fill_commands(new_commands)
    if new_commands and not quiet:
        print "New commands:"
        for command in new_commands:
            print command

    if kb_response and not quiet:
        print "KB response:", kb_response

    return (frames, new_commands, kb_response)


def _first_leaf(tree):
    """Return the first leaf of a tree with case normalized.
    >>> tree = Tree("(VP (VB Go) (PP-CLR (TO to) (NP-A (DT the) (NN hallway))))")
    >>> _first_leaf(tree)
    'go'
    >>> _first_leaf(tree[1])
    'to'
    >>> _first_leaf(tree[1][1])
    'the'

    """
    return tree.leaves()[0].lower()


def _immediate_children(tree):
    """Return the case-normalized leaves of from the children of a tree with only a single leaf.
    >>> tree = Tree("(S (NP-SBJ-A (-NONE- *)) " \
            "(VP (VB Go) (PP-CLR (TO to) (NP-A (DT the) (NN hallway))))(. .))")
    >>> _immediate_children(tree)
    ['*', '.']
    >>> _immediate_children(tree[1])
    ['go']

    """
    return [_first_leaf(subtree) for subtree in tree if len(subtree.leaves()) == 1]


def match_verbs(parse_tree, negated=False, subject=None):
    """Returns a list of matching verb frames for the given parse tree.

    Currently restricted to returning the first match to make things easier.
    """
    results = []
    condition = None

    # Descend into the tree to find verbs
    # If this is not rooted in an S, return nothing
    if not (main_tag(parse_tree.node).startswith(S_TAG) or main_tag(parse_tree.node) == VP_TAG):
        if MATCH_DEBUG:
            print "Not matching in tree rooted at {}".format(main_tag(parse_tree.node))
        return results

    # Check for conditionals by looking at the start of the tree
    conditional_marker = _first_leaf(parse_tree)
    if conditional_marker  in CONDITIONAL_MARKERS:
        s_children = [child for child in parse_tree if main_tag(child.node).startswith(S_TAG)]
        submatches = [match_verbs(s_child) for s_child in s_children]
        # Filter to non-none
        valid_submatches = [match for match in submatches if match]
        # If there are multiple submatches, we grab the first
        if valid_submatches:
            condition = (conditional_marker, valid_submatches[0])
            if MATCH_DEBUG:
                print "Conditional:", condition

    # Process VPs
    vps = [(child, parse_tree) for child in parse_tree if main_tag(child.node) == VP_TAG]
    for vp, parent in vps:
        # Check for support verbs (do, please)
        immed_children = _immediate_children(vp)
        support_verb = None
        for idx, child in enumerate(immed_children):
            if child in SUPPORT_VERBS:
                support_verb = (idx, child.lower())
                break

        if support_verb:
            idx, child = support_verb
            # Check for do not/don't
            child_negated = (child == DO_WORD and idx + 1 < len(vp) and
                             _first_leaf(vp[idx + 1]).lower() in NOT_WORDS)

            # Find the subject if not specified already
            if not subject:
                subject = _find_subject(parent)
                if not subject:
                    continue

            # Recurse on this vp
            results = match_verbs(vp, negated=child_negated, subject=subject)
            break
        else:
            # Check parent for negation
            for child in parent:
                # Never looks like: (ADVP-TMP (RB Never))
                if main_tag(child.node) == ADVP_TAG and _first_leaf(child) == NEVER_WORD:
                    negated = True
                    break

            # Check for verbs
            for child in vp:
                if child.node.startswith(VERB_TAG):
                    result_tree = (parent if not subject else Tree('S', [subject, vp]))
                    verb = child[0].lower()
                    match = best_matching_frame(verb, result_tree)
                    if match != (None, None):
                        match = FrameMatch(verb, match[1], match[0], negated, result_tree)
                        results.append(match)
                        # TODO: for now we just return the first match
                        break

    # Add the condition
    if condition:
        for match in results:
            match.condition_head, match.condition = condition

    return results


def _find_subject(parse_tree):
    """Return our best guess for what the subject is at the top level of a tree."""
    subjects = [child for child in parse_tree if child.node.startswith(SUBJ_TAG)]
    return subjects[0] if subjects else None


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
        if isinstance(subtree, Tree) and subtree[0] in AUXILIARY_VERBS:
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

    tag_match = re.compile(r'(?<=\()[A-Z][A-Z-]*')
    tags = tag_match.findall(str(parse_tree))
    if 'NP-PRD-A' not in tags and 'NP-SBJ' not in tags:
        # Only do WH movement if an NP predicate or NP subject exists:
        return parse_tree

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
    position = None

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
                if child.node not in ('CC', 'DT'):
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
        return {(0): ([parse_tree], '')}

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
# for position in parse_tree.treepositions():
# if not isinstance(parse_tree[position], Tree):
# continue
#
# subtree = parse_tree[position]
#
# if subtree.node == 'S':
# clause_dict[position] = (subtree, ' ')

    return clause_dict


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
    return ('WHADVP' in tree_string or
            'WHNP' in tree_string or
            'WHADJP' in tree_string or
            'WHPP' in tree_string or
            'SQ' in tree_string or
            'SINV' in tree_string or
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
    return ('SQ' in tree_string or
            'SINV' in tree_string or
            'SBARQ' in tree_string or
            '?' in tree_string)


if __name__ == "__main__":
    import doctest
    doctest.testmod()
