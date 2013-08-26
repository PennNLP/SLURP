#!/usr/bin/env python
"""Demonstration of the Penn NLP pipeline and semantics processing."""

from twisted.internet.protocol import ClientFactory
from twisted.protocols.basic import LineReceiver
from twisted.internet import reactor
from twisted.internet.task import LoopingCall
from pipelinehost import PipelineClient
from semantics.new_knowledge import KnowledgeBase
from semantics.parsing import process_parse_tree
from semantics.response import make_response
from pragbot.GameEnvironment import GameEnvironment
import semantics.parsing
import sys
import threading
import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer


class PragbotProtocol(LineReceiver):

    PRAGBOT_LISTEN_PORT = 20003

    def __init__(self):
        self.xmlrpc_server = SimpleXMLRPCServer(("localhost", self.PRAGBOT_LISTEN_PORT),
                                                logRequests=False, allow_none=True)
        self.xmlrpc_server.register_function(self.receiveHandlerMessages)
        self.xmlrpc_server_thread = threading.Thread(target=self.xmlrpc_server.serve_forever)
        self.xmlrpc_server_thread.daemon = True
        self.xmlrpc_server_thread.start()
        print "LTLMoPClient listening for XML-RPC calls on \
               http://localhost:{} ...".format(self.PRAGBOT_LISTEN_PORT)

        self.kb = KnowledgeBase(other_agents=['cmdr'])

    def receiveHandlerMessages(self, event_type, message):
        if event_type == "Move":
            room = self.ge.rooms[message]
            destination = room.center
            self.ge.jr.plan_path(destination)
        else:
            print event_type + " : " + message


    def sendMessage(self, action, msg):
        self.sendLine('%s%s' % (action, str(msg)))
        print 'Sending: %s%s' % (action, str(msg))
    def connectionMade(self):
        print 'Connected to server'

    def connectionLost(self, reason):
        print 'Server disconnected'
        reactor.stop()
    def dataReceived(self, data):
        lines = data.split('\n')
        print 'Received input: ', lines

        for s in lines:
            if s.startswith('CHAT_MESSAGE_PREFIX'):
                s = remove_prefix(s, 'CHAT_MESSAGE_PREFIX<Commander> ')
                # TODO: multi-process lock
                parse = PipelineClient().parse(s)
                frames, new_commands, kb_response = process_parse_tree(parse, data, self.kb, quiet=True)
                self.sendMessage('CHAT_MESSAGE_PREFIX', '<Junior> ' + make_response(new_commands, kb_response))
                self.ge.jr.plan_path(self.ge.cmdr.cell)
            elif s.startswith('MOVE_PLAYER_CELL'):
                s = remove_prefix(s, 'MOVE_PLAYER_CELL')
                new_x, old_x, new_y, old_y = s.split(',')
                self.ge.update_cmdr((int(new_x), int(new_y)))
                print str(self.ge)
            elif s.startswith('CREATE_FPSENVIRONMENT'):
                s = remove_prefix(s, 'CREATE_FPSENVIRONMENT')
                # This will be provided before environment related messages
                self.ge = GameEnvironment(s)
                lc = LoopingCall(self.ge.jr.follow_waypoints, self.sendMessage)
                lc.start(0.05)
def remove_prefix(s, prefix):
    return s[len(prefix):]

class PragbotFactory(ClientFactory):
    def buildProtocol(self, addr):
        return PragbotProtocol()

if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) == 2 else 10006
    print 'Initializing pragbot client...'
    reactor.connectTCP('localhost', port, PragbotFactory())
    reactor.run()
