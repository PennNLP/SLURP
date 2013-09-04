#!/usr/bin/env python
"""
Interface for debugging semantic frame matching.
"""

# Copyright (C) 2011-2013 Constantine Lignos, Ian Perera, and Kenton Lee
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

import sys

from semantics.frames import (_pick_best_match, _create_vfos)
from semantics.parsing import (match_verb, split_conjunctions)
from semantics.tree import Tree
from semantics.wntools import morphy
from pipelinehost import PipelineClient


HEADER_WIDTH = 72


def process(parse):
    """Show the steps of transformation for a parse."""
    # Original parse
    parse = Tree.parse(parse)
    print_parse(parse, "Parse")

    split_trees = split_conjunctions(parse)
    for split_parse, conjunction in split_trees.values():
        for subtree in split_parse:
            print_if_diff(subtree, parse, "Subtree ({})".format(conjunction))
            process_subtree(subtree)


def process_subtree(tree):
    """Follow the transformations for an individual subparse."""
    frame = match_verb(tree)
    print "Subtree for %s%s:" % (frame.verb, " (negated)" if frame.negated else "")
    print frame.tree
    print
    print "Arguments:"
    for position, tree in sorted(frame.args.items()):
        print "{}:".format(position), tree
    if frame.condition:
        print
        print "Condition:"
        print frame.condition.condition_head
        print frame.condition

def print_if_diff(new_parse, old_parse, heading):
    """Print new_parse if it is different from old_parse."""
    if new_parse != old_parse:
        print_parse(new_parse, heading)


def print_parse(parse, heading):
    """Print a tree in a pretty fashion."""
    # Print a centered header
    header_left = (HEADER_WIDTH - len(heading)) // 2
    header_right = HEADER_WIDTH - header_left
    print "-" * header_left + heading + "-" * header_right
    print parse.pprint(force_multiline=True)
    print


def main():
    """Get input, parse it, and pass it on to process."""
    client = PipelineClient(verbose=True)

    if len(sys.argv) > 1:
        text = " ".join(sys.argv[1:])
        parse = client.parse(text)
        if parse:
            process(parse)
        else:
            print "Connection to server closed."
    else:
        while True:
            try:
                text = raw_input('> ')
            except (KeyboardInterrupt, EOFError):
                break
            parse = client.parse(text)
            if parse:
                process(parse)
            else:
                print "Connection to server closed."
                break


if __name__ == "__main__":
    main()
