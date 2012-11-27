#!/usr/bin/env python
"""An interactive test client for the pipeline host."""

import socket
import sys
from semantics.processing import process_parse_tree
from semantics.new_knowledge import *
from pipelinehost import socket_parse, DEFAULT_PORT

def from_stdin(sock, kb):
    """Test semantics by processing text from stdin"""
    while True:
        try:
            text = raw_input('> ')
        except KeyboardInterrupt:
            break
        if not process_input(text, sock, kb):
            break

def process_input(text, sock, kb):
    """Send given text to the semantics component"""
    msg = socket_parse(asocket=sock, text=text)
    if msg:
        print msg
        semantics_result = process_parse_tree(msg, text, kb)
        print 'World knowledge:'
        print kb.readable()
        return True
    else:
        print 'Connection to server closed.'
        return False

if __name__ == "__main__":
    # Connect to the local socket and send
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('localhost', DEFAULT_PORT))

    kb = KnowledgeBase()

    if len(sys.argv) <= 1:
        from_stdin(sock, kb)
    else:
        process_input(" ".join(sys.argv[1:]), sock, kb)
