#!/usr/bin/env python
"""An interactive test client for the pipeline host."""

import sys

from pipelinehost import PipelineClient


def test():
    """Test by parsing some text."""
    client = PipelineClient(verbose=True)

    # If command line arguments are supplied, run them through
    if len(sys.argv) > 1:
        msg = client.parse(" ".join(sys.argv[1:]))
        if not msg:
            print "Connection to server closed."
        else:
            print msg
    else:
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
