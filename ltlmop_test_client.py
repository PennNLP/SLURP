#!/usr/bin/env python
"""
A simple connectivity test for the LTLMoPClient.

Running any actual

"""

# Copyright (C) 2011-2013 Constantine Lignos
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


from SimpleXMLRPCServer import SimpleXMLRPCServer
import threading

from pragbot import ltlmopclient

STARTING_LOCATION = "entrance"


class LTLMoPTestClient(object):
    """A simple testing client for testing LTLMoP and Pragbot together."""

    LISTEN_PORT = 20003

    def __init__(self):
        self.xmlrpc_server = SimpleXMLRPCServer(("localhost", self.LISTEN_PORT),
                                                logRequests=False, allow_none=True)
        self.xmlrpc_server.register_function(self.receiveHandlerMessages)
        self.xmlrpc_server_thread = threading.Thread(target=self.xmlrpc_server.serve_forever)
        self.xmlrpc_server_thread.daemon = True
        self.xmlrpc_server_thread.start()
        self.ltlmop = ltlmopclient.LTLMoPClient(self.LISTEN_PORT, self.display_chat)

        # Fake location status
        self.location = STARTING_LOCATION

    def get_user_input(self):
        """Get a line of input from the user."""
        user_text = raw_input('> ')
        if not user_text:
            return
        elif self.ltlmop.dialogue_manager is None:
            print "Error: Dialogue manager not initialized"
        else:
            self.ltlmop.on_receive_reply(self.ltlmop.dialogue_manager.tell(user_text))

    def receiveHandlerMessages(self, msg_type, msg=None):
        """Respond to a limited set of handler requests."""
        if msg_type == "Location":
            return self.location
        elif msg_type == "Sensor":
            return "0,0"
        elif msg_type == "Move":
            print "Motion: Moved to {}.".format(msg)
            self.location = msg
        else:
            if not msg:
                msg = "<No message>"
            print "{}: {}".format(msg_type, msg)

    def display_chat(self, msg):
        """Do nothing since the log displays these messages already."""
        pass


if __name__ == "__main__":
    LTLMOPTESTCLIENT = LTLMoPTestClient()
    while True:
        LTLMOPTESTCLIENT.get_user_input()
