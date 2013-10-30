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

try:
    import cPickle as pickle
except ImportError:
    import pickle
import os
import time
import matching
import treehandler
import copy

from collections import defaultdict
# We cannot use cElementTree because its output cannot be pickled.
from xml.etree.ElementTree import parse

from semantics.lexical_constants import UNDERSTOOD_SENSES
from semantics.wntools import morphy

from coordinating import Condition

PERF_DEBUG = False
WORD_SENSE_FILENAME = 'word_sense_mapping.pkl'
MODULE_PATH = os.path.dirname(os.path.abspath(__file__))
VERBNET_DIRECTORY = os.path.join(MODULE_PATH, 'Verbnet', 'verbnet-3.1')

# Mapping from Verbnet tags to Treebank tags
TAG_MAPPING = {'NP': ['NP'],
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
SYNRESTR_MAPPING = {'to_be': ('NP', 'to be', 'to_be', 'begins'),
                    'ac_ing': ('VP', 'gerund', 'ac_ing', 'VBG'),
                    'that_comp': ('S', 'that', 'that_comp', 'begins'),
                    'wh_comp': ('S', 'what', 'wh_comp', 'begins'),
                    'poss_ing': ('S', 'wanting', 'poss_ing', 'VBG'),
                    'wh_inf': ('S', 'how', 'wh_inf', 'WH')
                    }


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
            if not synrestr == '' and synrestr in SYNRESTR_MAPPING.keys():
                self.frame_list.append(SYNRESTR_MAPPING[synrestr])
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

    def match_parse_sequential(self,parse_tree,strict=0,allow_leftoverpps=2):
        '''Should replace match_parse...for now there should be no difference in the results returned
            
            Eventually:
            Takes a treebank parse tree compiled into NLTK's tree structure and
            sequentially matches this frame's verb frame. Non-sequential match_parse
            does not maintain the verbframe sequence. Not that the sequence is informative,
            but you never know.
            
            Also pops any SBAR-ADV and SBAR-TMP
        '''
        result_seq = []
        tree = copy.deepcopy(parse_tree)        
        matcher = matching.ParseMatcher(strict,allow_leftoverpps)
        matcher.th.pop_sbars(tree)
        matcher.th.depth_ulid_augment(tree, 0)
        match = matcher.match_frame(self.frame_list,tree)
        return match           
        
    def match_parse(self, parse_tree, strict=True, allow_leftovers=True):
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
                # TODO: Allow leftovers is completely broken
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


def get_verb_frames(verb):
    lemmatized_verb = morphy(verb, 'v')
    vfo_list = _create_vfos(lemmatized_verb)
    return vfo_list


def _create_vfos(word):
    """Gets VerbNet data without using NLTK corpora browser."""
    vfo_list = []
    for class_id in word_sense_mapping[word]:
        vfo_list.extend(sense_frame_mapping[class_id])
    return vfo_list


def best_matching_frame(verb, tree):
    """Get the best matching frame for a tree."""
    frames = get_verb_frames(verb)
    #matches = [frame.match_parse(tree) for frame in frames]
    matches = [frame.match_parse_sequential(tree) for frame in frames]
    valid_matches = [(args, frame.classid) for args, frame in zip(matches, frames) if args]
    best_match = _pick_best_match(valid_matches)
    return best_match


def _pick_best_match(match_list):
    """From the list of tuples, with the first element being the dict containing role assignments,
    and the second element being the verb class, pick the match that maps to an action if it exists."""
    if len(match_list) == 0:
        return (None, None)

    understood_matches = [(match, sense)
                          for match, sense in match_list if sense.split('-')[0] in UNDERSTOOD_SENSES]
    if len(understood_matches) > 0:
        return _pick_most_complete_match(understood_matches)
    # Otherwise use the all matches
    return _pick_most_complete_match(match_list)


def _pick_most_complete_match(match_list):
    longest = max(match_list, key=lambda x: len(x[0]))
    if sum(int(len(x) == longest) for x in match_list) > 1:
        return max(match_list, key=lambda x: int('Agent ' in x))
    else:
        return longest


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


# Load the Word/Sense/Frame maps on import
try:
    (word_sense_mapping, sense_frame_mapping) = load_word_sense_mapping()
except ValueError:
    (word_sense_mapping, sense_frame_mapping) = load_word_sense_mapping(True)
