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
from semantics.coordinating import Split

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
        self.sense = sense.split('-')[0]
        if not argdict:
            raise ValueError("Cannot create a FrameMatch with empty arguments")
        self.args = argdict
        self.negated = negated
        self.tree = tree
        self.condition = None
        self.condition_head = None

    def is_conditional(self):
        """Return whether this frame is conditioned on another frame."""
        return self.condition is None

    def is_negated(self):
        """Return whether this frame is negated."""
        return self.negated

    def __str__(self):
        condition = (self.condition if self.condition else
                     (self.condition_head if self.condition_head else None))
        return ("<FrameMatch verb: {} sense: {} args: {} negated: {} condition: {}>".format(
            self.verb, self.sense,
            ["{}: {}".format(arg, " ".join(tree.leaves())) for arg, tree in self.args.items()],
            self.negated, condition))

    def __repr__(self):
        return str(self)

    def pprint(self):
        """Return a pretty string representation."""
        pretty = "Verb: {}\nSense: {}\nArgs:\n{}\nNegated: {}\nConditioned: {}".format(
            self.verb, self.sense,
            "\n".join("\t{}: {}".format(arg, tree) for arg, tree in self.args.items()),
            self.negated,
            self.condition is not None)
        return pretty


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
        print "Error: semantics could not parse tree", repr(parse_tree_string)
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

    # TODO: This strange loop is because split_clauses may not work
    # for (clause, conjunction) in split_clause_dict.values():
    for clause, conjunction in ((parse_tree, ''),):
        # Split conjunctions and duplicate arguments if necessary
        #split_tree_dict = split_conjunctions(clause)
        split_tree_dict = split_conjunctions_sparse(clause)

        if conjunction != '':
            result_list.append(conjunction)

        for (split_tree, conjunction) in split_tree_dict.values():
            for tree in split_tree:
                if conjunction and verbose:
                    print "Subtree ({}):".format(conjunction)
                    print tree.pprint(force_multiline=True)
                # TODO: Deactivated for now
                # Store whether there was an existential there
                # if is_existential(str(tree)):
                #     tag_list.append('ex')

                # Transformational grammar stuff
                orig_tree = tree
                tree = existential_there_insertion(tree)
                tree = invert_clause(tree)
                tree = wh_movement(tree)

                if verbose and tree != orig_tree:
                    print 'Transformed tree:'
                    print tree.pprint(force_multiline=True)

                match = match_verb(tree, verbose=verbose)
                if match:
                    result_list.append(match)

    return result_list


def extract_entity(parse_tree, semantic_role='', verbose=False):
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

    # Frame semantic list is a list of conjunction strings and tuples, where the
    # first element of the tuple is the frame semantics, the second element is
    # the original tree branch it came from, the third is a list of tags, and
    # the fourth is the sense.
    for frame in frame_semantic_list:
        # Get the action associated with the sense
        # If such a mapping does not exist, use the original verb
        action = ACTION_ALIASES.get(frame.sense, frame.verb)

        item_to_entity = _framearg_entities(frame.args)

        # TODO: Make this more robust
        wh_question_type = get_wh_question_type(str(frame.tree))

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
        elif is_yn_question(str(frame.tree)):
            if 'Theme' in item_to_entity and 'Location' in item_to_entity:
                semantic_representation_list.append(
                    YNQuery(item_to_entity['Theme'], item_to_entity['Location']))
        # It's a regular command
        elif action is not None and action not in ('is', 'are', 'be'):
            current_command = _make_command(action, frame)
            # TODO: Figure out how to save the condition_head
            if frame.condition:
                #This seems to be a problem...the specgeneration expects conditions to be events
                #or assertions but here it looks like the only way conditions get set are as Commands
                #This also explains why the action of the command is a frame match
                #current_command.condition = _make_command(frame.condition, frame)
                frameaction = ACTION_ALIASES.get(frame.condition.sense, frame.condition.verb)
                current_command.condition = _make_command(frameaction, frame.condition)
            semantic_representation_list.append(current_command)
        # It's an assertion
        else:
            # TODO: Give a real value for whether it's an existential
            semantic_representation_list.append(
                Assertion(item_to_entity.get('Theme', None),
                          item_to_entity.get('Location', None), False))

    return semantic_representation_list


def _make_command(action, frame):
    """Make a command from a mapping of arguments."""
    item_to_entity = _framearg_entities(frame.args)
    theme = item_to_entity.get('Theme', None)
    agent = item_to_entity.get('Agent', None)
    patient = item_to_entity.get('Patient', None)
    if patient is None:
        patient = item_to_entity.get('Recipient', None)
    location = item_to_entity.get('Location', None)
    source = item_to_entity.get('Source', None)
    destination = item_to_entity.get('Destination', None)
    return Command(agent, theme, patient, location, source, destination, action,
                   negation=frame.negated)


def _framearg_entities(args):
    """Return a mapping with frame arguments replace by entities."""
    return {key: extract_entity(value, key) for key, value in args.items()}


def extract_commands(parse, knowledge_base=None, verbose=False):
    """Produces semantic interpretations of parse trees."""
    frames = extract_frames_from_parse(parse, verbose=verbose)
    semantic_structures = create_semantic_structures(frames)

    # TODO: Re-enable KB when everything else is stable
    # Update KB
    # if knowledge_base:
    #     kb_response = knowledge_base.process_semantic_structures(
    #         semantic_structures, source='cmdr')
    # else:
    #     kb_response = ''
    kb_response = None

    # Extract commands.
    new_commands = [item for item in semantic_structures if isinstance(item, Command)]
    # if knowledge_base:
    #     knowledge_base.fill_commands(new_commands)

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

def match_via_submatches(parse_tree,verbose=False):
    '''Search immediate children of parse_tree for conditional.
        If conditional is found, return
    '''
    # Look for a condition in a sub-S
    s_children = [child for child in parse_tree if main_tag(child.node).startswith(S_TAG)]
    submatches = [match_verb(s_child) for s_child in s_children]
    # Filter to non-none
    valid_submatches = [match for match in submatches if match]
    if valid_submatches:
        match = valid_submatches[0]
        # Check whether the current tree is a conditional by looking at the start of the tree
        first_word = _first_leaf(parse_tree)
        immed_children = _immediate_children(parse_tree)
        is_conditional = \
            immed_children and immed_children[0] == first_word and first_word in CONDITIONAL_MARKERS
       
        if is_conditional:
            match.condition_head = first_word
            if verbose:
                print "Returning conditional:", match
            return match, None
        else:
            # Check submatches for whether they are conditional
            if match.condition_head:
                # Set condition and then keep looking for VPs
                condition = match
                return None, condition
            else:
                # Otherwise, return what we've got so far
                return match, None
    return None, None
            
def match_verb(parse_tree, negated=False, subject=None, condition=None, verbose=False):
    """Returns a single matching verb frames for the given parse tree.

    Currently restricted to returning the first match to make things easier.
    """
    result = None
    condition = None
    if verbose:
        print "Matching on:"
        print parse_tree

    # Descend into the tree to find verbs
    # If this is not rooted in an S, return nothing
    if not (main_tag(parse_tree.node).startswith(S_TAG) or main_tag(parse_tree.node) == VP_TAG):
        if verbose:
            print "Not matching in tree rooted at {}".format(main_tag(parse_tree.node))
        return result

    match, condition = match_via_submatches(parse_tree,verbose)    
    if match:
        return match

    # Process VPs
    vps = [(child, parse_tree) for child in parse_tree if main_tag(child.node) == VP_TAG]
    for vp, parent in vps:
        #Look in VP for conditional
        if not condition: match,condition = match_via_submatches(vp,verbose)
        
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
            result = match_verb(vp, child_negated, subject, condition)
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
                        result = match
                        # TODO: for now we just return the first match
                        break

            # Break out if we've found something
            if result:
                break

    # Add the condition
    if condition:
        if result:
            result.condition = condition
        else:
            return condition

    return result


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


def split_conjunctions_sparse(parse_tree):
    '''Find conjunctions, split them based on syntax tree. Return the result dictionary
    '''
    splitter = Split()    
    trees = splitter.split_on_cc(parse_tree)
    #Abstracting away path to conjunction because it isn't used anyway
    #Only support 'and' right now
    res = {}
    res[0] = (trees,"and")
    return res
    
def split_conjunctions(parse_tree):
    """Find conjunctions in a given parse_tree, and create a list of parse trees
    with the conjunctions removed and the conjoined clauses in place of the
    parent node of the conjunction.
    
    """
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
