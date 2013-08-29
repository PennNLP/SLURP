#!/usr/bin/env python
"""An individual SLURP client for the Pragbot game."""

# Copyright (C) 2013 Israel Geselowitz and Constantine Lignos
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

import signal
import threading
import time
import socket
from socket import error, timeout
from SimpleXMLRPCServer import SimpleXMLRPCServer

from semantics.new_knowledge import KnowledgeBase
from pragbot.GameEnvironment import GameEnvironment
from pragbot import ltlmopclient
from pragbot.ltlmopclient import find_port

PRAGBOT_SERVER_PORT = 10006
# Because this is at the highest risk for a race condition, handler needs
# the highest port of all the XML-RPC ports.
HANDLER_BASE_PORT = 13000


class PragbotClient(object):
    """Provide a SLURP client for the Pragbot server."""

    KNOWN_OBJECTS = set(("bomb", "hostage", "badguy"))
    EVENT_LOCATION = "Location"
    EVENT_MOVE = "Move"
    EVENT_STOP = "Stop"
    EVENT_SENSOR = "Sensor"
    EVENT_FIND_BOMB = "Find Bomb"
    LOCATION_UNKNOWN = "Unknown"

    def __init__(self):
        # Set up basics before starting the server
        self.delimiter = "\n"
        self.ge = None
        self.is_ready = threading.Event()
        self.kb = KnowledgeBase(other_agents=['cmdr'])

        # Connect to the pragbot server first
        try:
            self._conn = socket.create_connection(('localhost', PRAGBOT_SERVER_PORT))
        except error:
            raise IOError("Could not connect to Pragbot server on port " + str(PRAGBOT_SERVER_PORT))

        # Find a port for the handlers
        self.handler_port = find_port(HANDLER_BASE_PORT)
        print "Using port {} for handlers".format(self.handler_port)

        # Start the RPC server for handler requests
        self.xmlrpc_server = SimpleXMLRPCServer(("localhost", self.handler_port),
                                                logRequests=False, allow_none=True)
        self.xmlrpc_server.register_function(self.receiveHandlerMessages)
        self.xmlrpc_server_thread = threading.Thread(target=self.xmlrpc_server.serve_forever)
        self.xmlrpc_server_thread.daemon = True

        # Start the server and then kick off the LTLMoP client which will indirectly connect to it
        self.xmlrpc_server_thread.start()
        print "LTLMoPClient listening for XML-RPC calls on \
               http://localhost:{} ...".format(self.handler_port)
        self.ltlmop = ltlmopclient.LTLMoPClient(self.handler_port)

    def receiveHandlerMessages(self, event_type, message=None):
        """Process messages from the handlers."""
        # Wait for the environment to be loaded
        self.is_ready.wait()

        # Skip location messages since we get them so often
        if event_type != self.EVENT_LOCATION:
            print "Received handler message:", event_type, message

        # Handle events
        if event_type == self.EVENT_MOVE:
            room = self.ge.rooms[message]
            destination = room.center
            self.ge.jr.set_waypoints([])
            self.ge.jr.plan_path(self.ge.grid[destination[0]][destination[1]])
        elif event_type == self.EVENT_STOP:
            self.ge.jr.set_waypoints([])
        elif event_type in self.KNOWN_OBJECTS:
            object_seen = False
            for thing in self.ge.objects:
                if thing.startswith(event_type):
                    object_seen = (object_seen or
                                   self.ge.object_positions[thing] in self.ge.rooms[message])
            return object_seen
        elif event_type == self.EVENT_FIND_BOMB:
            if message in self.ge.objects:
                return self.ge.object_positions[message]
            else:
                return None
        elif event_type == self.EVENT_LOCATION:
            for room in self.ge.rooms.itervalues():
                if self.ge.jr.cell.location in room:
                    return room.name
            return self.LOCATION_UNKNOWN
        else:
            print event_type + " : " + message

    def sendMessage(self, action, msg):
        """Send a message to the server."""
        data = action + str(msg)
        self._conn.sendall(data + self.delimiter)
        print 'Sending: ', data

    def process_line(self, line):
        """Process a line from the server."""
        if line.startswith('CHAT_MESSAGE_PREFIX'):
            line = _remove_prefix(line, 'CHAT_MESSAGE_PREFIX<Commander> ')
            # TODO: multi-process lock
            self.ltlmop.get_pragbot_input(line)
        elif line.startswith('MOVE_PLAYER_CELL'):
            line = _remove_prefix(line, 'MOVE_PLAYER_CELL')
            new_x, old_x, new_y, old_y = line.split(',')
            self.ge.update_cmdr((int(new_x), int(new_y)))
            print str(self.ge)
        elif line.startswith('CREATE_FPSENVIRONMENT'):
            line = _remove_prefix(line, 'CREATE_FPSENVIRONMENT')
            # This will be provided before environment related messages
            self.ge = GameEnvironment(line)
            self.is_ready.set()
            call = RepeatingCall(self.ge.jr.follow_waypoints, (self.sendMessage,), 0.05)
            call.daemon = True
            call.start()

    def run(self):
        """Process requests from the server."""
        buff = ""
        while True:
            try:
                buff += self._conn.recv(4096)
            except timeout:
                continue
            except error:
                break
            if not buff:
                break
            while buff:
                msg, buff = _parse_msg(buff, self.delimiter)
                if msg:
                    self.process_line(msg)
                else:
                    break

        print "Server disconnected"

    def shutdown(self):
        """Close any resources before exiting."""
        print "Shutting down PragbotClient..."
        # Close down the socket
        try:
            self._conn.shutdown(socket.SHUT_RDWR)
        except error:
            pass
        self._conn.close()


class RepeatingCall(threading.Thread):
    """Repeatedly call a function in a separate thread."""

    def __init__(self, func, args, interval):
        self.func = func
        self.args = args
        self.interval = interval
        threading.Thread.__init__(self)

    def run(self):
        """Call the function in a loop."""
        while True:
            self.func(*self.args)
            time.sleep(self.interval)


def _parse_msg(buff, delim):
    """Split a buffer into the msg and the remaining data."""
    idx = buff.find(delim)
    if idx != -1:
        # Throw away the delim when slicing by adding its length
        return (buff[:idx], buff[idx + len(delim):])
    else:
        # Return no message, only buffer
        return (None, buff)


def _remove_prefix(s, prefix):
    """Remove a prefix from the start of a string."""
    return s[len(prefix):]


def main():
    """Create a pragbot client."""
    print 'Initializing pragbot client...'
    client = PragbotClient()

    # Set up signal handlers
    def shutdown(signum, frame):  # pylint:disable=W0613
        """Stop the reactor."""
        print "Shutting down client..."
        client.shutdown()

    signal.signal(signal.SIGABRT, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    print 'Running pragbot client...'
    try:
        client.run()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
