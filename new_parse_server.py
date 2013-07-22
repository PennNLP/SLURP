#!/usr/bin/env python
"""Demonstration of the Penn NLP pipeline and semantics processing."""

from pipelinehost import PipelineClient
from twisted.internet import protocol, reactor
from semantics.new_knowledge import KnowledgeBase
from semantics.parsing import process_parse_tree
from semantics.response import make_response
from semantics import tree
import semantics.parsing
import json
import sys
import threading
import time

SECRET_CODE = ",oO-i2De<2W5NVuJa6E"
LOG_FILE = "parse_server_log.txt"

class PipelineProtocol(protocol.Protocol):
    def __init__(self, kb, lock):
        self.kb = kb
        self.lock = lock

    def dataReceived(self, data):
        # Remove extra escape characters
        data = data.replace('\\','')
        print 'Received input: ', data

        knowledge_demo = data.startswith(SECRET_CODE)
        if knowledge_demo:
            data = data[len(SECRET_CODE):]

        with open(LOG_FILE, 'a') as f:
            f.write('%s | %s | "%s"\n' % (time.asctime(), str(self.transport.getPeer()), data))

        with self.lock:
            parse = PipelineClient(verbose=True).parse(data)

        response = {}
        frames, new_commands, kb_response = process_parse_tree(parse, data, self.kb if knowledge_demo else None, quiet=True)
        response['parse'] = parse
        if frames is not None:
            # We do a join and split to make sure all whitespace becomes single spaces
            modified_trees = [" ".join(str(modified_parse_tree[1]).split())
                              for modified_parse_tree in frames
                              if (len(modified_parse_tree) > 1 and
                                  isinstance(modified_parse_tree[1], tree.Tree))]
            response['trees'] = list(set(modified_trees))
            response['frames'] = [frame_dict for frame_dict in [frame[0] for frame in frames
                                                    if isinstance(frame[0], dict)]]
        else:
            response['trees'] = []
            response['frames'] = []
        response['response'] = make_response(new_commands, kb_response)
        response['structures'] = '\n\n'.join(str(c) for c in new_commands)
        self.transport.write(json.dumps(response))

class PipelineFactory(protocol.Factory):
    def __init__(self):
        self.kb = KnowledgeBase(other_agents=['cmdr'])
        self.lock = threading.Lock()

    def buildProtocol(self, addr):
        return PipelineProtocol(self.kb, self.lock)

def main(port):
    print 'Initializing parse server...'
    reactor.listenTCP(port, PipelineFactory())    
    reactor.run()

if __name__ == '__main__':
    main(int(sys.argv[1]) if len(sys.argv) == 2 else 10001)
