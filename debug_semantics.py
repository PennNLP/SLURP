#!/usr/bin/env python
"""An interactive test client for the pipeline host."""

import socket
import sys
from semantics.processing import process_parse_tree
from pipelinehost import socket_parse, DEFAULT_PORT


def from_stdin(sock):
    """Test by parsing some text with semantics"""
    while True:
        try:
            text = raw_input('> ')
        except KeyboardInterrupt:
            break
        if not process_input(text, sock):
            break
def process_input(text, sock):
    msg = socket_parse(asocket=sock, text=text)
    if msg:
        print msg
        user_response, semantics_result, semantics_response, new_commands = process_parse_tree(msg, text)
        return True
    else:
        print "Connection to server closed."
        return False
if __name__ == "__main__":
    # Connect to the local socket and send
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('localhost', DEFAULT_PORT))

    if len(sys.argv) <= 1:
        from_stdin(sock)
    else:
        process_input(" ".join(sys.argv[1:]), sock)
