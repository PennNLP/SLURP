#!/usr/bin/env python
"""Provide an interface to LTLMoP execution."""

import os
import sys
import socket
import multiprocessing
import threading
import time
import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer

import project
from specCompiler import SpecCompiler
from LTLParser.LTLParser import Parser
import execute

from ltlbroom import talkback

RESPONSE_PLANNING = "Okay, just a moment while I make a plan."

# TODO: This class with one instance is overkill
# pylint: disable=W0201
class Config(object):
    executor_base_port = 11000
    ltlmop_base_port = 12000
    max_port_tries = 100

    def __init__(self):
        self.base_spec_dir = "pragbotscenario"
        self.base_spec = "pragbot.spec"

    def get_spec_file(self):
        return os.path.join(self.base_spec_dir, self.base_spec)


def find_port(base):
    """Try to find an open port starting at base."""
    port = base
    while (port - base) < Config.max_port_tries:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('127.0.0.1', port))
        except socket.error:
            port += 1
        else:
            sock.close()
            return port


class LTLMoPClient(object):
    RESPONSE_DELIM = "RESPONSE_DELIM"  # Split response by words and semantics

    def __init__(self, handler_port, chat_callback):
        self.CONFIG = Config()
        self.handler_port = handler_port
        self.map_bitmap = None
        self.robot_pos = None
        self.fiducial_positions = {}
        self.chat_callback = chat_callback
        # To be set by load_project
        self.proj = None
        self.executor_proxy = None
        self.dialogue_manager = None
        self.load_project(False)

    def load_project(self, reloading):
        """Load a project from CONFIG.spec_file, reusing the proxy if reloading."""
        spec_file = self.CONFIG.get_spec_file()
        self.proj = project.Project()
        self.proj.loadProject(spec_file)
        # Set the compiler to give structured responses
        self.proj.compile_options["slurp_struct_responses"] = True
        if not reloading:
            self.executor_proxy = self.get_executor(spec_file)
        else:
            self.executor_proxy.initialize(spec_file, None)
        self.dialogue_manager = self.get_dialogue_manager()

    def get_dialogue_manager(self):
        # Start dialogue manager
        dialogue_manager = BarebonesDialogueManager(self, self.executor_proxy)

        self.user_name = "User"

        # Wait for executor to fully boot
        self.append_log(">> Please wait, initializing...")
        while not self.executor_ready_flag.wait(0.1):
            time.sleep(.1)
        self.append_log(">> Ready!")
        self.append_log("  ")

        # Tell the user we are ready
        self.append_log("Hello.", "System")

        return dialogue_manager

    def get_executor(self, spec_file):
        # Start execution context
        print "Starting executor..."
        self.executor_ready_flag = threading.Event()
        self.executor_port = find_port(self.CONFIG.executor_base_port)
        print "Using port {} for executor".format(self.executor_port)
        self.executor_process = multiprocessing.Process(
            target=execute.execute_main,
            args=(self.executor_port,
                  spec_file,
                  None, False, {'handler_port': self.handler_port}))
        self.executor_process.start()

        # Start our own xml-rpc server to receive events from execute
        self.server_port = find_port(self.CONFIG.ltlmop_base_port)
        print "Using port {} for LTLMoP client".format(self.server_port)
        self.xmlrpc_server = SimpleXMLRPCServer(
            ("127.0.0.1", self.server_port),
            logRequests=False, allow_none=True)

        # Register functions with the XML-RPC server
        self.xmlrpc_server.register_function(self.handleEvent)

        # Kick off the XML-RPC server thread
        self.xmlrpc_server_thread = threading.Thread(
            target=self.xmlrpc_server.serve_forever)
        self.xmlrpc_server_thread.daemon = True
        self.xmlrpc_server_thread.start()
        print "LTLMoPClient listening for XML-RPC calls on \
               http://127.0.0.1:{} ...".format(self.server_port)

        # Connect to executor
        print "Connecting to executor...",
        while True:
            try:
                executor_proxy = xmlrpclib.ServerProxy(
                    "http://127.0.0.1:{}".format(self.executor_port),
                    allow_none=True)

                # Register with executor for event callbacks
                executor_proxy.registerExternalEventTarget(
                    "http://127.0.0.1:{}".format(self.server_port))
            except socket.error:
                sys.stdout.write(".")
            else:
                break
        print
        return executor_proxy

    def set_project(self, specfile):
        """Set the project of this client via a new specfile"""
        self.CONFIG.base_spec = specfile
        self.load_project(True)

    def handleEvent(self, event_type, event_data):
        """Processes messages from the controller, and updates the GUI accordingly"""

        if event_type in ["FREQ"]:  # Events to ignore
            pass
        elif event_type == "POSE":
            self.robot_pos = event_data
        elif event_type == "FID":
            if event_data[1] is None:
                # Hide the fiducial
                del self.fiducial_positions[event_data[0]]
            else:
                # Update fiducial position
                self.fiducial_positions[event_data[0]] = event_data[1:]
        elif event_type == "MESSAGE":
            # Provide a way for any part of LTLMoP to give feedback
            self.append_log(event_data, "System")
        elif event_type == "READY":
            self.executor_ready_flag.set()
        else:
            print "[{}] {}".format(event_type, event_data)

    def shutdown(self):
        print "Shutting down executor..."
        try:
            self.executor_proxy.shutdown()
        except socket.error:
            # Executor probably crashed
            pass

        self.xmlrpc_server.shutdown()
        self.xmlrpc_server_thread.join()
        print "Waiting for executor to quit..."
        print "(If this takes more than 5 seconds it has crashed and \
                you will need to run `killall python` for now)"
        self.executor_process.join(5)
        # After ten seconds, just kill it
        if self.executor_process.is_alive():
            self.executor_process.terminate()
            print "Executor did not exit in time and was executed."
        else:
            print "Executor exited normally."

    def append_log(self, message, agent=None):
        message = str(message)
        if agent:
            print agent + " : " + message
        else:
            print message

    def get_pragbot_input(self, user_text):  # wxGlade: LTLMoPClient.<event_handler>
        # echo
        self.append_log(user_text, self.user_name)
        # response
        if self.dialogue_manager is None:
            self.append_log("Dialogue manager not initialized", "!!! Error")
        else:
            try:
                reply = self.dialogue_manager.tell(user_text)
            except Parser.ParseErrors:
                self.append_log("LTLParser encountered an error.")
                reply = self.dialogue_manager.interpreter.CRASH
            except IOError:
                self.append_log("Could not connect to NLPipeline.")
                raise
            else:
                if type(reply) == list:
                    [self.on_receive_reply(w) for w in reply]
                else:
                    self.on_receive_reply(reply)

    def on_receive_reply(self, result):
        """ when the dialoguemanager has gotten back to us """
        if result:
            self.chat_callback(result)
            self.append_log(result, "System")

    def on_clear_commands(self, event):
        self.dialogue_manager.clear()
        event.Skip()

    def on_execute(self, event):
        self.dialogue_manager.execute()
        event.Skip()

# end of class LTLMoPClient


class BarebonesDialogueManager(object):

    # GOTIT responsibility was on specgen but now it is on the dialogue manager
    GOTIT = "Got it. I can {!r}"
    DEFAULT_SPECGEN_PROBLEM = "I'm sorry, I didn't quite understand that."
    SPECIFIC_SPECGEN_PROBLEM = "Could not understand"

    def __init__(self, ltlmopclient, executor, base_spec=None):
        """ take reference to execution context and gui_window
            optionally initialize with some base spec text """

        self.ltlmop = ltlmopclient
        self.executor = executor

        if base_spec is None:
            self.base_spec = []
        else:
            self.base_spec = base_spec.split("\n")

        self.spec = []

        # Initiate a specCompiler to hang around and give us immediate parser
        # feedback
        self.compiler = SpecCompiler()
        self.compiler.proj = self.ltlmop.proj

        self.chat_dict = {"clear_actions": {"prompts": ["clear"],
                                            "response": ("clear_command", "")},
                          "activate_actions": {"prompts": ["go", "activate", "execute"],
                                               "response": ("activate_command", "")},
                          "pause_actions": {"prompts": ["wait", "stop"],
                                            "response": ("no_act", "Stopping until you tell me to start " +
                                                         "or give me new commands.")},
                          "status_requests": {"prompts": ["status"],
                                              "response": ("status_request", "")},
                          "resume_actions": {"prompts": ["start", "begin"],
                                             "response": ("resume_act", "Picking up where I left off.")},
                          "speclist_requests": {"prompts": ["list"],
                                                "response": ("speclist_request", "")},
                          "non_actionable_chats": {"prompts": ["hello", "hi", "how's it going?"],
                                                   "response": ("chatbot_response", "Hi!")}
                          }
        self.paused = False

        # Set up talkback
        self.interpreter = talkback.FriendlyResponseInterpreter()


    def clear(self):
        self.spec = []
        self.ltlmop.append_log("Cleared the specification.", "System")

    def execute(self):
        # TODO: don't resynthesize if the specification hasn't changed?
        # i.e. distinguish between resuming from pause, versus a new command

        if not self.spec:
            return "You haven't given me any orders I understand."

        # pause
        self.executor.pause()

        # Let the user know we're synthesizing
        self.ltlmop.on_receive_reply(RESPONSE_PLANNING)

        # Trigger resynthesis
        spec = self.get_spec()
        success = self.executor.resynthesizeFromNewSpecification(spec)
        if success:
            # TODO: Remove this to allow carryover of commands when
            # resynthesizing
            self.clear()
            # resume
            self.executor.resume()
            return "Got it. I'm carrying out your orders."
        else:
            self.clear()
            return ("Sorry, I can't come up with a plan that will carry out all your orders. "
                    "Try giving fewer commands at a time.")

    def handle_chats(self, message):
        """Return the response code and text if appropriate"""
        msg = message.lower().strip().strip('.')
        for entry in self.chat_dict:
            if msg in self.chat_dict[entry]["prompts"]:
                return self.chat_dict[entry]["response"]
        return "other", ""

    def tell(self, message):
        """ take in a message from the user, return a response.
            WARNING: this is effectively running in non-main thread"""

        response_code, chat = self.handle_chats(message)
        if response_code == "clear_command":
            self.clear()
            return
        elif response_code == "activate_command":
            if not self.paused:
                return self.execute()
            self.paused = False
            self.executor.resume()
            return "Picking up where I left off."
        elif response_code == "no_act":
            if not self.paused and self.executor.isRunning():
                self.paused = True
                self.executor.pause()
                return chat
            elif not self.paused:
                return "I wasn't doing anything so I cannot stop or pause."
            return "Already paused execution."
        elif response_code == "resume_act":
            if self.paused:
                self.paused = False
                self.executor.resume()
                return chat
            return self.execute()
        elif response_code == "status_request":
            if not self.executor.isRunning():
                return chat
            curr_goal_num = self.executor.getCurrentGoalNumber()
            if curr_goal_num is None:
                return chat
            else:
                return self.executor.getCurrentGoalDescription()
        elif response_code == "chatbot_response":
            return chat
        elif response_code == "speclist_request":
            return "\n".join(self.spec)
        elif response_code == "other":
            # Ask parser if this individual line is OK
            # FIXME: Because _writeLTLFile() is so monolithic, this will
            # clobber the `.ltl` file
            # FIXME: This may only work with SLURP
            message = message.strip()
            self.compiler.proj.specText = message
            spec, traceback_tree, responses = self.compiler._writeLTLFile()
            if spec is not None:
                self.spec.append(message)

            return [self.interpreter.interpret(response) for command_responses in responses
                            for response in command_responses]

    def _error_on_specgen(self, reponse):
        return reponse.startswith(self.SPECIFIC_SPECGEN_PROBLEM) or reponse == self.DEFAULT_SPECGEN_PROBLEM

    def get_spec(self):
        """ return the current specification as one big string """
        return "\n".join(self.base_spec + self.spec)
