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
        self._location_lock = threading.Lock()
        self._location = STARTING_LOCATION

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
            return self.get_location()
        elif msg_type == "Sensor":
            return "0,0"
        elif msg_type == "Move":
            print "Motion: Moving to {}.".format(msg)
            self.move(msg)
        else:
            if not msg:
                msg = "<No message>"
            print "{}: {}".format(msg_type, msg)

    def display_chat(self, msg):
        """Do nothing since the log displays these messages already."""
        pass

    def get_location(self):
        """Get the current location."""
        with self._location_lock:
            return self._location

    def set_location(self, value):
        """Set the current location."""
        with self._location_lock:
            self._location = value

    def move(self, location):
        """Move the robot in a background thread after a delay."""
        def move_delay():
            """Move to a lexically enclosed location."""
            self.set_location(location)
            print "Motion: Moved to {}.".format(location)
        mover = threading.Timer(3.0, move_delay)
        mover.start()


if __name__ == "__main__":
    LTLMOPTESTCLIENT = LTLMoPTestClient()
    while True:
        LTLMOPTESTCLIENT.get_user_input()
