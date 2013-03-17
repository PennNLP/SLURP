#!/usr/bin/env python
"""Demonstration of the Penn NLP pipeline and semantics processing."""

from twisted.internet.protocol import Factory
from twisted.protocols.basic import LineReceiver
from twisted.internet import reactor
from pipelinehost import PipelineClient
from semantics.new_knowledge import KnowledgeBase
from semantics.parsing import process_parse_tree
from semantics.response import make_response
from pragbot.GameEnvironment import GameEnvironment
import semantics.parsing
import sys
import threading

class PragbotProtocol(LineReceiver):
    def __init__(self, game_id, lock):
        self.game_id = game_id
        self.lock = lock
        self.kb = KnowledgeBase(other_agents=['cmdr'])
        self.ge = GameEnvironment()

    def sendMessage(self, action, msg):
        self.sendLine('%s%s' % (action, str(msg)))

    def connectionMade(self):
        print 'Client connected'
        self.sendMessage('SET_ROLE', 1)
        self.sendMessage('CREATE_FPSENVIRONMENT', self.ge.to_fps_string())
        self.sendMessage('CREATE_ENVIRONMENT', self.ge.to_string())

    def connectionLost(self, reason):
        print 'Client disconnected'

    def dataReceived(self, data):
        lines = data.split('\n')
        print 'Received input: ', lines

        for s in lines:
            if s.startswith('CHAT_MESSAGE_PREFIX'):
                s = s[len('CHAT_MESSAGE_PREFIX<Commander> '):]
                with self.lock:
                    parse = PipelineClient().parse(s)
                frames, new_commands, kb_response = process_parse_tree(parse, data, self.kb, quiet=True)
                self.sendMessage('CHAT_MESSAGE_PREFIX', '<Junior> ' + make_response(new_commands, kb_response))

class PragbotFactory(Factory):
    def __init__(self):
        self.count = 0 # Set this based on logs
        self.lock = threading.Lock()
        print 'Ready for clients...'

    def buildProtocol(self, addr):
        pp = PragbotProtocol(self.count, self.lock)
        self.count += 1
        return pp

if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) == 2 else 1235    
    print 'Initializing pragbot server...'
    reactor.listenTCP(port, PragbotFactory())    
    reactor.run()
