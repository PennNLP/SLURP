#!/usr/bin/env python
"""An interactive test client for the pipelin host."""

import socket

from pipelinehost import socket_parse, DEFAULT_PORT


def test():
    """Test by parsing some text."""
    # Connect to the local socket and send
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('localhost', DEFAULT_PORT))
    while True:
        try:
            text = raw_input('> ')
        except KeyboardInterrupt:
            break
        msg = socket_parse(asocket=sock, text=text)
        if not msg:
            print "Connection to server closed."
            break
        print msg


if __name__ == "__main__":
    test()