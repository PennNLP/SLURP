#!/usr/bin/env python
"""
Hosts requests sent to the NLPipeline over sockets.

"""

# Copyright (C) 2012 Constantine Lignos
#
# This file is a part of SLURP.
#
# SLURP is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# SLURP is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with SLURP.  If not, see <http://www.gnu.org/licenses/>.

import json
import socket
from commproxy import CallbackSocket, _parse_msg
from pennpipeline import PennPipeline

MSG_SEP = "\n"
DEFAULT_PORT = 9001


def _socket_parse(**kwargs):
    """Send a remote parsing request and get a response."""
    # Get the socket out of kwargs and send over the rest
    sock = kwargs.pop('asocket')
    msg = json.dumps(kwargs) + MSG_SEP
    print "Sending:", repr(msg)
    sock.sendall(msg)
    print "Waiting for response..."
    buff = sock.recv(4096)
    msg, buff = _parse_msg(buff, MSG_SEP)
    return msg


class PipelineHost(CallbackSocket):
    """Provides a connection to pipeline over a listening socket."""
    name = "pipelinehost"

    def __init__(self, port, local=False):
        # Set up callback
        CallbackSocket.__init__(self, port, MSG_SEP, local)
        self.register_callback(self.parse_text)
        # Start up the pipeline
        self.pipeline = PennPipeline()

    def parse_text(self, message):
        """Receive a parse request for the pipeline."""
        try:
            data = json.loads(message)
        except ValueError:
            # Make a default set of arguments
            data = {'text': message}
        print "Message:", repr(data)
        print "Parsing..."
        # pylint: disable=W0142,E1101
        response = self.pipeline.parse_text(**data)
        print "Sending:", repr(response)
        self.send(response)


class PipelineClient(object):
    """Provides a client to the PipelineHost."""

    def __init__(self, port=DEFAULT_PORT, hostname='localhost'):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((hostname, port))
        except socket.error:
            raise IOError("Could not connect to pipelinehost on %s:%d. "
                          "Make sure that pipelinehost is running." %
                          (hostname, port))

    def parse(self, text, force_nouns=None, force_verbs=None):
        """Parse text using a remote pipeline."""
        # Lowercase and strip any trailing punctuation.
        text = text.strip().lower()
        # Wrap in kwargs
        return _socket_parse(asocket=self.sock, text=text, force_nouns=force_nouns,
                             force_verbs=force_verbs)

    def close(self):
        """Close the connection to the pipeline host.

        If you need the connection closed promptly, it's wise to call this, but garbage collection
        will accomplish the same purpose.
        """
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()


def main():
    """Start the parsing server and leave it up until KeyboardInterrupt."""
    PipelineHost(DEFAULT_PORT, True)
    try:
        while True:
            raw_input()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
