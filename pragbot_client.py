#!/usr/bin/env python
"""Demonstration of the Penn NLP pipeline and semantics processing."""

import sys
import threading
from SimpleXMLRPCServer import SimpleXMLRPCServer

import twisted
from twisted.internet.protocol import ClientFactory
from twisted.protocols.basic import LineReceiver
from twisted.internet import reactor
from twisted.internet.task import LoopingCall

from pipelinehost import PipelineClient
from semantics.new_knowledge import KnowledgeBase
from semantics.parsing import process_parse_tree
from semantics.response import make_response
from pragbot.GameEnvironment import GameEnvironment
from pragbot import ltlmopclient


class PragbotProtocol(LineReceiver):

    PRAGBOT_LISTEN_PORT = 20003
    KNOWN_OBJECTS = set(("bomb", "hostage", "badguy"))

    def __init__(self):
        # Set up basics before starting the server
        self.delimiter = "\n"
        self.ge = None
        self.is_ready = threading.Event()
        self.kb = KnowledgeBase(other_agents=['cmdr'])

        self.xmlrpc_server = SimpleXMLRPCServer(("localhost", self.PRAGBOT_LISTEN_PORT),
                                                logRequests=False, allow_none=True)
        self.xmlrpc_server.register_function(self.receiveHandlerMessages)
        self.xmlrpc_server_thread = threading.Thread(target=self.xmlrpc_server.serve_forever)
        self.xmlrpc_server_thread.daemon = True

        # Start the server and then the client which will connect to it
        self.xmlrpc_server_thread.start()
        print "LTLMoPClient listening for XML-RPC calls on \
               http://localhost:{} ...".format(self.PRAGBOT_LISTEN_PORT)
        self.ltlmop = ltlmopclient.LTLMoPClient()

    def receiveHandlerMessages(self, event_type, message=None):
        print "Received handler message:", event_type, message
        self.is_ready.wait()
        if event_type == "Move":
            room = self.ge.rooms[message]
            destination = room.center
            self.ge.jr.plan_path(destination)
        elif event_type == "Stop":
            self.ge.jr.plan_path(self.ge.jr.cell)
        elif event_type in self.KNOWN_OBJECTS:
            object_seen = False
            for thing in self.ge.objects:
                if thing.startswith(event_type):
                    object_seen = (object_seen or
                                   self.ge.object_positions[thing] in self.ge.rooms[message])
            return object_seen
        elif event_type == "Find Bomb":
            if message in self.ge.objects:
                return self.ge.object_positions[message]
            else:
                return None
        elif event_type == "Location":
            for room in self.ge.rooms:
                if self.ge.jr.cell in room:
                    return room
            return "Nowhere"
        else:
            print event_type + " : " + message

    def sendMessage(self, action, msg):
        self.sendLine('%s%s' % (action, str(msg)))
        print 'Sending: %s%s' % (action, str(msg))

    def connectionMade(self):
        print 'Connected to server'

    def connectionLost(self, reason):
        print 'Server disconnected'
        try:
            reactor.stop()
        except twisted.internet.error.ReactorNotRunning:
            pass

    def lineReceived(self, line):
        print 'Received input: {!r}'.format(line)
        if line.startswith('CHAT_MESSAGE_PREFIX'):
            line = remove_prefix(line, 'CHAT_MESSAGE_PREFIX<Commander> ')
            # TODO: multi-process lock
            self.ltlmop.get_pragbot_input(line)
        elif line.startswith('MOVE_PLAYER_CELL'):
            line = remove_prefix(line, 'MOVE_PLAYER_CELL')
            new_x, old_x, new_y, old_y = line.split(',')
            self.ge.update_cmdr((int(new_x), int(new_y)))
            print str(self.ge)
        elif line.startswith('CREATE_FPSENVIRONMENT'):
            line = remove_prefix(line, 'CREATE_FPSENVIRONMENT')
            # This will be provided before environment related messages
            self.ge = GameEnvironment(line)
            self.is_ready.set()
            lc = LoopingCall(self.ge.jr.follow_waypoints, self.sendMessage)
            lc.start(0.05)


def remove_prefix(s, prefix):
    return s[len(prefix):]


class PragbotFactory(ClientFactory):

    def buildProtocol(self, addr):
        return PragbotProtocol()


def main():
    """Create a pragbot client."""
    port = 10006
    print 'Initializing pragbot client...'
    reactor.connectTCP('localhost', port, PragbotFactory())
    reactor.run()


if __name__ == '__main__':
    main()
