#!/usr/bin/env python
"""An interactive test client for the pipelin host."""

from pipelinehost import PipelineClient


def test():
    """Test by parsing some text."""
    client = PipelineClient(verbose=True)
    while True:
        try:
            text = raw_input('> ')
        except (KeyboardInterrupt, EOFError):
            break
        msg = client.parse(text)
        if not msg:
            print "Connection to server closed."
            break
        print msg


if __name__ == "__main__":
    test()
