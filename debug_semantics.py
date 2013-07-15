#!/usr/bin/env python
"""An interactive test client for the pipeline host."""

import sys
from semantics.parsing import process_parse_tree
import semantics.parsing
from semantics.new_knowledge import KnowledgeBase
from semantics.response import make_response
from pipelinehost import PipelineClient

semantics.parsing.EXTRACT_DEBUG = True
TEST_CASES_FILE = 'semantics/test_cases.txt'


def from_stdin(kb):
    """Test semantics by processing text from stdin"""
    while True:
        try:
            text = raw_input('> ')
        except KeyboardInterrupt:
            break
        if not process_input(text, kb):
            break

def process_input(text, kb, verbose=True):
    """Send given text to the semantics component"""
    msg = PipelineClient().parse(text)
    if msg:
        print msg
        frames, new_commands, kb_response = process_parse_tree(msg, text, kb, quiet=True)
        if verbose:
            print "Frames: %s" % '\n'.join(str(f) for f in frames);
            print "Knowledge base: %s" % str(kb)
        
        print 'Response: %s' % make_response(new_commands, kb_response)
        print '\n'.join(str(c) for c in new_commands)
        return True
    else:
        print 'Connection to server closed.'
        return False

def run_test_cases(kb):
    semantics.parsing.EXTRACT_DEBUG = False
    with open(TEST_CASES_FILE) as f:
        utterances = f.readlines()
    for u in utterances:
        process_input(u, kb, verbose=False)
        print '_'*65

if __name__ == "__main__":
    KB = KnowledgeBase(other_agents=['cmdr'])

    if len(sys.argv) <= 1:
        from_stdin(KB)
    elif len(sys.argv) == 2 and sys.argv[1] == 'test':
        run_test_cases(KB)
    else:
        process_input(" ".join(sys.argv[1:]), KB)
