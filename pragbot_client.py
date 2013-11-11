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
import logging

from semantics.new_knowledge import KnowledgeBase
from pragbot.game import GameEnvironment
from pragbot import ltlmopclient
from pragbot.ltlmopclient import find_port
import globalConfig  # pylint: disable=W0611

DEBUG_MOVE = False

PRAGBOT_SERVER_PORT = 10006
# Because this is at the highest risk for a race condition, handler needs
# the highest port of all the XML-RPC ports.
HANDLER_BASE_PORT = 13000

RESPONSE_CRASH = ("Sorry, my language understanding system isn't working. "
                  "I'm afraid we can't perform our mission.")


class PragbotClient(object):
    """Provide a SLURP client for the Pragbot server."""

    KNOWN_OBJECTS = set(("bomb", "hostage", "badguy"))
    EVENT_LOCATION = "Location"
    EVENT_MOVE_LOCATION = "Move_Location"
    OBJECT_LOCATION = "Object_Location"
    EVENT_MOVE = "Move"
    EVENT_STOP = "Stop"
    EVENT_SENSOR = "Sensor"
    EVENT_FIND_BOMB = "Find Bomb"
    LOCATION_UNKNOWN = "Unknown"
    EVENT_DEFUSING = "Defusing"
    EVENT_DEFUSE = "Defuse"
    
    JR_STATE = "JR_STATE"
    BOMBS_STATE = "bombs"
    BOMB_SENSOR = "bomb"

    def __init__(self):
        # Set up basics before starting the server
        self.delimiter = "\n"
        self.ge = None
        self.is_ready = threading.Event()
        self.kb = KnowledgeBase(other_agents=['cmdr'])
        self._waypoint_thread = None
        self.stop = False
        self.shutting_down = False
        self.defusing = False
        self.sensor_states ={} 

        # Connect to the pragbot server first
        try:
            self._conn = socket.create_connection(('127.0.0.1', PRAGBOT_SERVER_PORT))
        except error:
            raise IOError("Could not connect to Pragbot server on port " + str(PRAGBOT_SERVER_PORT))

        # Find a port for the handlers
        self.handler_port = find_port(HANDLER_BASE_PORT)
        logging.info("Using port {} for handlers".format(self.handler_port))

        # Start the RPC server for handler requests
        self.xmlrpc_server = SimpleXMLRPCServer(("127.0.0.1", self.handler_port),
                                                logRequests=False, allow_none=True)
        self.xmlrpc_server.register_function(self.receiveHandlerMessages)
        self.xmlrpc_server_thread = threading.Thread(target=self.xmlrpc_server.serve_forever)
        self.xmlrpc_server_thread.daemon = True

        # Start the server and then kick off the LTLMoP client which will indirectly connect to it
        self.xmlrpc_server_thread.start()
        logging.info("Handler host listening for XML-RPC calls on http://127.0.0.1:{} ...".
                     format(self.handler_port))
        self.ltlmop = ltlmopclient.LTLMoPClient(self.handler_port, self.send_response)

    def receiveHandlerMessages(self, event_type, message=None):
        """Process messages from the handlers."""
        # Wait for the environment to be loaded
        self.is_ready.wait()

        # Skip location and sensormessages since we get them so often
        if event_type != self.EVENT_LOCATION and event_type != self.EVENT_SENSOR:
            logging.info("Received handler message: %s %s", event_type, message)

        # Handle events
        if event_type == self.EVENT_MOVE:
            room = self.ge.rooms[message]
            destination = room.center
            self.move_jr(destination)
        elif event_type == self.EVENT_MOVE_LOCATION:
            #Move to destination
            destination = tuple(message)
            self.move_jr(destination)
        elif event_type == self.OBJECT_LOCATION:
            if message == self.BOMBS_STATE:                             
                return self.sensor_states[message]
            else:
                logging.warning("Unknown location sensor: %s ", message)
        elif event_type == self.EVENT_STOP:
            if not self.defusing:
                #If defusing, ignore EVENT_STOP commands
                self.clear_jr_waypoints()
                logging.info("Movement waypoints cleared.")
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
        elif event_type == self.EVENT_DEFUSE:
            # TODO: Make bomb disappear here, not when we hit it 

            #Remove bomb from sensor states
            destination = tuple(message)
            destination_string = ',' + str(destination[0]) + ':' + destination[1] + ','
            if destination_string in self.sensor_states:
                self.sensor_states.replace(destination_string,",")
            else:
                logging.warning("Unable to defuse sensor_states at location: %s ", message)
            
        elif event_type == self.EVENT_LOCATION:
            for room in self.ge.rooms.itervalues():
                if self.ge.jr.cell.location in room:
                    if message == "coordinates":
                        return self.ge.jr.cell.location
                    return room.name
            return self.LOCATION_UNKNOWN        
        elif event_type == self.EVENT_SENSOR:
            if message == self.BOMB_SENSOR:
                #the bomb sensor checks the "bombs" state                    
                return self.sensor_states[self.BOMBS_STATE]
            elif message in self.sensor_states:
                return self.sensor_states[message]
            logging.warning("Unknown sensor: %s ", message)                    
            return False   
        elif event_type == self.EVENT_DEFUSING:
            if message == "True":
                self.defusing = True
            elif message == "False":
                self.defusing = False
            else:
                logging.warning("Unknown defusing status value: %s ", message)
        else:
            logging.warning("Unknown event_type: %s %s", event_type, message)

    def sendMessage(self, action, msg):
        """Send a message to the server."""
        if self.stop:
            # We're shutting down already, don't even try to send
            return

        data = action + str(msg)
        # Skip player move messages
        if DEBUG_MOVE or not data.startswith("PLAYER_MOVE"):
            logging.info('Sending: %s', data)

        # Send, but give up if the connection is dead
        try:
            self._conn.sendall(data + self.delimiter)
        except error:
            # Shut down, the connection is broken
            logging.error("Could not send data to server, shutting down.")
            self.stop = True

    def process_line(self, line):
        """Process a line from the server."""
        if line.startswith('CHAT_MESSAGE_PREFIX'):
            line = _remove_prefix(line, 'CHAT_MESSAGE_PREFIX<Commander> ')
            logging.info("Received chat message: %s", line)
            # TODO: multi-process lock
            try:
                self.ltlmop.get_pragbot_input(line)
            except IOError:
                # Error connecting to NLPipeline
                self.ltlmop.on_receive_reply(RESPONSE_CRASH)
                logging.error("Pipelinehost cannot be reached, shutting down.")
                self.stop = True
                self.shutdown()
            except Exception as err:
                logging.exception("Error when processing user input.")
                self.ltlmop.on_receive_reply(RESPONSE_CRASH)
                self.stop = True
                self.shutdown()
        elif line.startswith('MOVE_PLAYER_CELL'):
            line = _remove_prefix(line, 'MOVE_PLAYER_CELL')
            new_x, old_x, new_y, old_y = line.split(',')
            self.ge.update_cmdr((int(new_x), int(new_y)))
            # Uncomment for a crazy amount of logging, showing a map every single move
            # logging.debug(self.ge)
        elif line.startswith('CREATE_FPSENVIRONMENT'):
            line = _remove_prefix(line, 'CREATE_FPSENVIRONMENT')
            # This will be provided before environment related messages
            self.ge = GameEnvironment(line)
            logging.info("Scenario map:\n%s", self.ge)
            self.is_ready.set()
            self._waypoint_thread = \
                RepeatingCall(self.ge.jr.follow_waypoints, (self.sendMessage,), 0.05)
            self._waypoint_thread.daemon = True
            self._waypoint_thread.start()
        elif line.startswith('JR_IS_FLIPPED') or line.startswith('JR_IS_UNFLIPPED'):
            logging.info("Received JR flipped message: %s", line)
            self.ge.jr.flip_junior()
        elif line.startswith(self.JR_STATE):
            #JR_STATEroom=12;name= cellar;bombs=0;hostages=0;badguys=0
            line = _remove_prefix(line,self.JR_STATE)            
            self.set_sensor_states(line)            
            logging.info("Received JR state message: %s",line)
            
    def set_sensor_states(self,line):
        """set the sensor states from a state update."""
        pieces = [w.replace(" ","") for w in line.split(";")]
        for key, value in [w.split("=") for w in pieces]:            
            self.sensor_states[key] = value        
            
    def move_jr(self,destination):
        """Move Junior to destination"""
        logging.info("Moving JR to destination: "+str([destination]))
        self.clear_jr_waypoints()
        self.ge.jr.plan_path(self.ge.grid[destination[0]][destination[1]])

    def clear_jr_waypoints(self):
        """Clear Junior's waypoints"""
        self.ge.jr.set_waypoints([])

    def send_response(self, msg):
        """Send a chat response to the server."""
        self.sendMessage('CHAT_MESSAGE_PREFIX<JR> ', msg)

    def run(self):
        """Process requests from the server."""
        buff = ""
        while not self.stop:
            try:
                buff += self._conn.recv(4096)
            except timeout:
                logging.debug("Timed out waiting for data.")
                continue
            except error:
                logging.warning("Error reading data from server.")
                break
            if not buff:
                logging.warning("Server closed connection.")
                break
            while buff:
                msg, buff = _parse_msg(buff, self.delimiter)
                if msg:
                    self.process_line(msg)
                elif msg is None:
                    # Incomplete message, recv again for more data
                    break
                else:
                    # Adjacent delimiters in the input, keep going
                    continue
        # Set self.stop in case we came out from a break, and then shutdown
        self.stop = True
        self.shutdown()

    def shutdown(self):
        """Close any resources before exiting."""
        if self.shutting_down:
            # No need to call this multiple times
            return
        self.shutting_down = True

        logging.info("Shutting down PragbotClient...")
        # Stop the waypoint callback thread
        if self._waypoint_thread:
            logging.info("Waiting for waypoint thread to stop...")
            self._waypoint_thread.stop = True
            self._waypoint_thread.join()

        logging.info("Shutting down LTLMoPClient...")
        self.ltlmop.shutdown()

        logging.info("Shutting down handler server...")
        self.xmlrpc_server.shutdown()
        self.xmlrpc_server_thread.join()

        # Close down the socket
        try:
            self._conn.shutdown(socket.SHUT_RDWR)
        except error:
            pass
        self._conn.close()
        logging.info("PragbotClient has shut down.")


class RepeatingCall(threading.Thread):
    """Repeatedly call a function in a separate thread."""

    def __init__(self, func, args, interval):
        self.func = func
        self.args = args
        self.interval = interval
        self.stop = False
        threading.Thread.__init__(self)

    def run(self):
        """Call the function in a loop."""
        while not self.stop:
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
        """Stop the client."""
        print "Shutting down client..."
        client.stop = True
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
