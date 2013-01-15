#!/usr/bin/env python
"""An interactive test client for the pipeline host."""

import sys
from semantics.processing import process_parse_tree
from semantics.new_knowledge import KnowledgeBase
from pipelinehost import PipelineClient

def from_stdin(kb):
    """Test semantics by processing text from stdin"""
    while True:
        try:
            text = raw_input('> ')
        except KeyboardInterrupt:
            break
        if not process_input(text, kb):
            break

def process_input(text, kb):
    """Send given text to the semantics component"""
    msg = PipelineClient().parse(text)
    if msg:
        print msg
        process_parse_tree(msg, text, kb)
        print 'World knowledge:'
        print kb.readable()
        return True
    else:
        print 'Connection to server closed.'
        return False

if __name__ == "__main__":
    KB = KnowledgeBase()

    if len(sys.argv) <= 1:
        from_stdin(KB)
    else:
        process_input(" ".join(sys.argv[1:]), KB)
