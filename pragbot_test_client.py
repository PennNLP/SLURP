#!/usr/bin/env python
"""Demonstration of the Penn NLP pipeline and semantics processing."""

from pragbot_client import PragbotProtocol


class PragbotTestProtocol(PragbotProtocol):

    def receiveHandlerMessages(self, event_type, message=None):
        print "Event type:", event_type
        print "Message:", message

    def sendMessage(self, action, msg):
        print 'Sending:'
        print 'Action:', action
        print 'Message:', msg


if __name__ == '__main__':
    PragbotTestProtocol()
    raw_input()
