from SimpleXMLRPCServer import SimpleXMLRPCServer
import threading

from pragbot import ltlmopclient

class LTLMoPTestClient(object):
    
    LISTEN_PORT = 20003

    def __init__(self):
        self.xmlrpc_server = SimpleXMLRPCServer(("127.0.0.1", self.LISTEN_PORT),
                                                logRequests=False, allow_none=True)
        self.xmlrpc_server.register_function(self.receiveHandlerMessages)
        self.xmlrpc_server_thread = threading.Thread(target=self.xmlrpc_server.serve_forever)
        self.xmlrpc_server_thread.daemon = True
        self.xmlrpc_server_thread.start()
        self.ltlmop = ltlmopclient.LTLMoPClient()
        
    def get_user_input(self):  # wxGlade: LTLMoPClient.<event_handler>
        user_text = raw_input('> ')
        if user_text == "":
            return
        # response
        if self.ltlmop.dialogue_manager is None:
            print "Dialogue manager not initialized", "!!! Error"
        else:
            self.ltlmop.on_receive_reply(self.ltlmop.dialogue_manager.tell(user_text))
        
    def receiveHandlerMessages(self, msg_type, msg=None):
        if(msg == None):
            msg = "Blank"
        print msg_type + ": " + msg

if __name__ == "__main__":
    LTLMOPTESTCLIENT = LTLMoPTestClient()
    # Change this variable to switch from test mode to regular mode
    while True:
        LTLMOPTESTCLIENT.get_user_input()
        
