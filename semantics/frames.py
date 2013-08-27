"""Converts parse tree representation into a tree that can be matched to Verbnet
frames, and then returns the matching frames and their corresponding trees."""

# Copyright (C) 2011-2013 Eric Doty, Kenton Lee, Constantine Lignos, and Ian Perera
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
try:
    import cPickle as pickle
except ImportError:
    import pickle
import os
import time
from copy import deepcopy
from collections import defaultdict
# We cannot use cElementTree because its output cannot be pickled.
from xml.etree.ElementTree import parse

from semantics.lexical_constants import UNDERSTOOD_SENSES
from semantics.tree import Tree

PERF_DEBUG = False
WORD_SENSE_FILENAME = 'word_sense_mapping.pkl'
MODULE_PATH = os.path.dirname(os.path.abspath(__file__))
VERBNET_DIRECTORY = os.path.join(MODULE_PATH, 'Verbnet', 'verbnet-3.1')

# Parse tree constants
VP_TAG = "VP"
S_TAG = "S"
SUBJ_TAG = "NP-SBJ"
VERB_TAG = "VB"
ADVP_TAG = "ADVP"
DO_WORD = "do"
SUPPORT_VERBS = set((DO_WORD, "please"))
NOT_WORDS = set(("not", "n't"))
NEVER_WORD = "never"


class VerbFrame(object):

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

    def match_parse(self, parse_tree, strict=True, allow_leftovers=False):
        """Takes a Treebank parse tree compiled into NLTK's tree structure.
        Outputs a result dictionary mapping predicates to arguments"""

        result_dict = {}
        matches = 0
        skipped_NP = 0

        vp_idx = 0
        pp_idx = 0
        pp_subtree = None
        vp_subtree = None
        obj_location = None
        first_NP = True
        nonstrict_PP = False

        for frame in self.frame_list:
            if not vp_subtree or vp_idx < len(vp_subtree):  # vp_subtree is empty or isn't at its end

                simple_role = self._simple_frame(frame, first_NP)

                if simple_role == "subject":
                    first_NP = False
                    subtrees = list(parse_tree)
                    for subtree in subtrees:
                        if self._simple_tag(subtree) == "NP":
                            result_dict[frame[1]] = subtree
                            matches += 1
                        elif self._simple_tag(subtree) == "VP" and matches > 0:
                            vp_subtree = subtree
                            obj_location = "VP"
                            break

                elif simple_role == "verb":
                    if vp_subtree:
                        for subtree in vp_subtree:
                            if self._simple_tag(subtree).startswith("VB"):
                                result_dict[frame[1]] = subtree
                                vp_idx += 1
                                matches += 1
                                break
                            vp_idx += 1
                    else:
                        return None

                elif simple_role == "prep":
                    if vp_subtree:  # makes sure prep is inside VP (e.g. PP Location VERB Agent should(?)/will fail)
                        for subtree in vp_subtree[vp_idx:]:  # go through arguments of VP looking for PP
                            if self._simple_tag(subtree) == "PP":
                                if self._simple_tag(subtree[0]) == "PP":  # if first tag in PP is another PP
                                    if pp_subtree and pp_idx == len(pp_subtree):
                                        pp_idx = 0
                                    pp_subtree = subtree[pp_idx]
                                    pp_idx += 1
                                else:
                                    pp_subtree = subtree

                                if (frame[1] == "in" and pp_subtree[0].node == "IN" or
                                        frame[1] == "to towards" and pp_subtree[0].node == "TO" or
                                        frame[1] == "PREP"):
                                    result_dict[frame[1]] = pp_subtree[0]
                                    matches += 1
                                    obj_location = "PP"
                                    break
                                else:
                                    return None

                    elif not strict and pp_subtree:
                        if (frame[1] == "in" and pp_subtree[0].node == "IN" or
                                frame[1] == "to towards" and pp_subtree[0].node == "TO" or
                                frame[1] == "PREP"):
                            result_dict[frame[1]] = pp_subtree[0]
                            matches += 1
                            obj_location = "PP"
                        else:
                            return None
                    else:
                        return None

                elif simple_role == "object":
                    if obj_location == "PP":
                        if len(pp_subtree) > 1 and self._simple_tag(pp_subtree[1]) == "NP":
                            matches += 1
                            if not strict and not nonstrict_PP and vp_idx + 1 == len(vp_subtree) and len(pp_subtree[1]) > 1 and self._simple_tag(pp_subtree[1][1]) == "PP" and matches < len(self.frame_list):
                                result_dict[frame[1]] = pp_subtree[1][0]
                                pp_subtree = pp_subtree[1][1]
                                vp_subtree = None
                                nonstrict_PP = True
                                skipped_NP += 1
                            else:
                                result_dict[frame[1]] = pp_subtree[1]
                                if pp_idx == 0:
                                    vp_idx += 1
                                obj_location = "VP"
                    elif obj_location == "VP":
                        if self._simple_tag(vp_subtree[vp_idx]) == "NP":
                            result_dict[frame[1]] = vp_subtree[vp_idx]
                            matches += 1
                            vp_idx += 1
                        else:
                            return None
                else:
                    return None
            else:
                return None

        # Check results
        if result_dict:
            # Check that every frame is filled
            if len(result_dict) == len(self.frame_list) and matches == len(self.frame_list):
                if not allow_leftovers:
                    # Count relevant tags in result_dict
                    no_leftover_list = [
                        'SBAR-A', 'SINV', 'NP', 'NP-SBJ-A', 'NP-A', 'IN', 'TO']  # nodes that must be in frame output
                    result_count = 0
                    original_count = 0
                    for frame_tree in result_dict.values():
                        frame_subtrees = list(frame_tree.subtrees())
                        for frame_subtree in frame_subtrees:
                            for tag in no_leftover_list:
                                if tag == frame_subtree.node:
                                    result_count += 1
                    # Count tags in original subtree
                    original_subtree_list = list(parse_tree.subtrees())
                    for orig_subtree in original_subtree_list:
                        for tag in no_leftover_list:
                            if tag == orig_subtree.node:
                                original_count += 1
                    if (result_count + skipped_NP) >= original_count:
                        return result_dict
                else:
                    return result_dict
        else:
            return None

    def _simple_frame(self, frame_tag, first_NP=False):
        if frame_tag[0] == 'NP':
            if first_NP:
                return "subject"
            else:
                return "object"
        elif frame_tag[0] == 'PREP':
            return "prep"
        elif frame_tag[0] == 'VERB':
            return "verb"
        else:
            return "other"

    def _simple_tag(self, subtree):
        return subtree.node.split("-")[0]


# Mapping from Verbnet tags to Treebank tags
tag_mapping = {'NP': ['NP'],
               #('NP', 'Location'): ['PP-LOC', 'PP-DIR', 'PP-CLR', 'NN', 'ADVP-LOC', 'NP-A', 'WHADVP', 'ADVP-TMP', 'PP-PRD', 'ADVP-DIR'],
               #('NP', 'Destination'): ['PP-LOC', 'PP-DIR', 'NN', 'NP-A', 'ADVP', 'PP-CLR', 'WHADVP', 'ADVP-DIR'],
               ('NP', 'Location'): ['NN', 'ADVP-LOC', 'NP-A', 'WHADVP', 'ADVP-TMP', 'ADVP-DIR'],
               ('NP', 'Destination'): ['NN', 'NP-A', 'ADVP', 'WHADVP', 'ADVP-DIR'],
               ('NP', 'Source'): ['NN', 'NP-A', 'ADVP', 'WHADVP', 'ADVP-DIR'],  # Eric: added NP-A (Not sure why?)
               ('NP', 'Asset'): ['NP-A'],
               ('NP', 'Agent'): ['NP-SBJ-A', 'NP', 'NP-A'],
               ('NP', 'Beneficiary'): ['NP-A'],
               ('NP', 'Recipient'): ['NP-A'],
               ('NP', 'Patient'): ['NP'],
               ('NP', 'Instrument'): ['NP-A'],
               ('NP', 'Topic'): ['S-A', 'NP-A', 'WHNP'],
               ('NP', 'Theme'): ['NP-A', 'NP-SBJ-A', 'NP', 'NP-SBJ',
                                 'WHNP', 'WP'],
               ('PREP',): ['exact'],
               'PREP': ['IN', 'TO', 'ADVP-DIR'],
               'VERB': ['VB']  # , 'VBZ', 'VBP', 'VBD']
               }

# Mapping from syntax restrictions to Treebank tags and matching conditions
synrestr_mapping = {'to_be': ('NP', 'to be', 'to_be', 'begins'),
                    'ac_ing': ('VP', 'gerund', 'ac_ing', 'VBG'),
                    'that_comp': ('S', 'that', 'that_comp', 'begins'),
                    'wh_comp': ('S', 'what', 'wh_comp', 'begins'),
                    'poss_ing': ('S', 'wanting', 'poss_ing', 'VBG'),
                    'wh_inf': ('S', 'how', 'wh_inf', 'WH')
                    }

auxiliary_verbs = ['was', 'is', 'get', 'are', 'got', 'were', 'been', 'being']


def load_word_sense_mapping(force_generate=False):
    """Loads the pickle file for mapping words to VerbNet frames. *Required*
    for other functions to work."""

    word_sense_path = os.path.join(VERBNET_DIRECTORY, WORD_SENSE_FILENAME)
    try:
        # We use the existing exception handling (designed to handle a missing file) for cases
        # where the caller wants to require that things be generated from
        # scratch.
        if force_generate:
            raise IOError("Forcing generation of new pickle file")

        # Try to open the pickled file, allowing us to fail fast if
        # there's no pickle file
        word_sense_file = open(word_sense_path, 'rb')

        # Otherwise, confirm that the pickled file is up to date
        # Get the most recent file in the directory
        max_mtime = 0
        for dirname, _, files in os.walk(VERBNET_DIRECTORY):
            for filename in files:
                # We only care about the XML and pickle files
                if not (filename.endswith('.xml') or filename.endswith('.pkl')):
                    continue
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
        if PERF_DEBUG:
            tic = time.time()
        result = pickle.load(word_sense_file)
        if PERF_DEBUG:
            print 'Time taken to load pickle file: %f' % (time.time() - tic)
        word_sense_file.close()
    except (IOError, AttributeError, EOFError):
        # IOError: no such file, AttributeError: API change that makes file unloadable,
        # EOFError: corrupt/empty file
        print "Word sense pickle file is missing or out of date, creating it..."
        result = generate_mapping(word_sense_path)

    return result


def fill_mappings(node, temp_word_sense_mapping, temp_sense_frame_mapping):
    class_id = node.attrib['ID']
    for member in node.findall('MEMBERS/MEMBER'):
        temp_word_sense_mapping[member.attrib['name']].add(class_id)
    for frame in node.findall('FRAMES/FRAME/SYNTAX'):
        temp_sense_frame_mapping[class_id].append(VerbFrame(class_id, node, list(frame)))


def generate_mapping(result_filename):
    """Create and store the mappings between senses and words."""
    file_list = os.listdir(VERBNET_DIRECTORY)
    temp_word_sense_mapping = defaultdict(set)
    temp_sense_frame_mapping = defaultdict(list)
    if PERF_DEBUG:
        tic = time.time()
    for filename in file_list:
        if (filename[-4:] == '.xml'):
            try:
                with open(os.path.join(VERBNET_DIRECTORY, filename), 'r') as verbnet_file:
                    tree = parse(verbnet_file)
                    fill_mappings(
                        tree.getroot(), temp_word_sense_mapping, temp_sense_frame_mapping)
                    for subclass in tree.getroot().findall('SUBCLASSES/VNSUBCLASS'):
                        fill_mappings(
                            subclass, temp_word_sense_mapping, temp_sense_frame_mapping)
            except IOError:
                continue

    if PERF_DEBUG:
        print 'Time taken to parse xml files: %f' % (time.time() - tic)

    with open(result_filename, 'wb') as pickle_file:
        pickle.dump((temp_word_sense_mapping, temp_sense_frame_mapping),
                    pickle_file, pickle.HIGHEST_PROTOCOL)

    return (temp_word_sense_mapping, temp_sense_frame_mapping)

try:
    (word_sense_mapping, sense_frame_mapping) = load_word_sense_mapping()
except ValueError:
    (word_sense_mapping, sense_frame_mapping) = load_word_sense_mapping(True)


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


def _first_leaf(tree):
    """Return the first leaf of a tree with case normalized."""
    return tree.leaves()[0].lower()


def _immediate_children(tree):
    """Return the leaves of from the children of a tree that have only a single leaf."""
    return [_first_leaf(subtree) for subtree in tree if len(subtree.leaves()) == 1]


def find_verbs(parse_tree, negated=False, subject=None):
    """Returns the list of tuples: (verb, negation)"""
    results = []

    # Descend into the tree to find verbs
    # If this is not rooted in an S, return nothing
    if not (parse_tree.node.startswith(S_TAG) or parse_tree.node.startswith(VP_TAG)):
        return results

    # Process embedded S
    s_children = [child for child in parse_tree if child.node.startswith(S_TAG)]
    for s_child in s_children:
        results.extend(find_verbs(s_child))

    # Process VPs
    vps = [(child, parse_tree) for child in parse_tree if child.node.startswith(VP_TAG)]
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
            results.extend(find_verbs(vp, negated=child_negated, subject=subject))
            break
        else:
            # Check parent for negation
            for child in parent:
                # Never looks like: (ADVP-TMP (RB Never))
                if child.node.startswith(ADVP_TAG) and _first_leaf(child) == NEVER_WORD:
                    negated = True
                    break

            # Check for verbs
            for child in vp:
                if child.node.startswith(VERB_TAG):
                    result_tree = (parent if not subject else Tree('S', [subject, vp]))
                    results.append((child[0].lower(), negated, result_tree))

    return results


def _find_subject(parse_tree):
    """Return our best guess for what the subject is at the top level of a tree."""
    subjects = [child for child in parse_tree if child.node.startswith(SUBJ_TAG)]
    return subjects[0] if subjects else None


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


def create_vfos(word):
    """Gets VerbNet data without using NLTK corpora browser."""
    vfo_list = []
    for class_id in word_sense_mapping[word]:
        vfo_list.extend(sense_frame_mapping[class_id])
    return vfo_list


def pick_best_match(match_list):
    """From the list of tuples, with the first element being the dict containing role assignments,
    and the second element being the verb class, pick the match that maps to an action if it exists."""
    if len(match_list) == 0:
        return (None, None)

    understood_matches = [(match, sense)
                          for match, sense in match_list if sense.split('-')[0] in UNDERSTOOD_SENSES]
    if len(understood_matches) > 0:
        return pick_most_complete_match(understood_matches)
    # Otherwise use the all matches
    return pick_most_complete_match(match_list)


def pick_most_complete_match(match_list):
    longest = max(match_list, key=lambda x: len(x[0]))
    if sum(int(len(x) == longest) for x in match_list) > 1:
        return max(match_list, key=lambda x: int('Agent ' in x))
    else:
        return longest


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
