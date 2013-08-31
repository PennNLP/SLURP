#!/usr/bin/env python

import os
import sys
import getpass
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

# TODO: This class with one instance is overkill
# pylint: disable=W0201
class Config(object):
    pass
CONFIG = Config()
CONFIG.base_spec_file = os.path.join("pragbotscenario", "pragbot.spec")
CONFIG.executor_base_port = 11000
CONFIG.ltlmop_base_port = 12000
CONFIG.max_port_tries = 100

RESPONSE_ERROR = "Sorry, something went wrong when I tried to understand that."


def find_port(base):
    """Try to find an open port starting at base."""
    port = base
    while (port - base) < CONFIG.max_port_tries:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(('localhost', port))
        except socket.error:
            port += 1
        else:
            sock.close()
            return port


class LTLMoPClient(object):

    def __init__(self, handler_port, chat_callback):
        self.map_bitmap = None
        self.robot_pos = None
        self.fiducial_positions = {}
        self.proj = project.Project()
        self.proj.loadProject(CONFIG.base_spec_file)
        self.chat_callback = chat_callback

        # Start execution context
        print "Starting executor..."
        self.executor_ready_flag = threading.Event()
        self.executor_port = find_port(CONFIG.executor_base_port)
        print "Using port {} for executor".format(self.executor_port)
        self.executor_process = multiprocessing.Process(
            target=execute.execute_main,
            args=(self.executor_port,
                  self.proj.getFilenamePrefix() + ".spec",
                  None, False, {'handler_port': handler_port}))
        self.executor_process.start()

        # Start our own xml-rpc server to receive events from execute
        self.server_port = find_port(CONFIG.ltlmop_base_port)
        print "Using port {} for LTLMoP client".format(self.server_port)
        self.xmlrpc_server = SimpleXMLRPCServer(("localhost", self.server_port),
                                                logRequests=False, allow_none=True)

        # Register functions with the XML-RPC server
        self.xmlrpc_server.register_function(self.handleEvent)

        # Kick off the XML-RPC server thread
        self.xmlrpc_server_thread = threading.Thread(target=self.xmlrpc_server.serve_forever)
        self.xmlrpc_server_thread.daemon = True
        self.xmlrpc_server_thread.start()
        print "LTLMoPClient listening for XML-RPC calls on \
               http://localhost:{} ...".format(self.server_port)

        # Connect to executor
        print "Connecting to executor...",
        while True:
            try:
                self.executor_proxy = xmlrpclib.ServerProxy(
                    "http://localhost:{}".format(self.executor_port),
                    allow_none=True)

                # Register with executor for event callbacks
                self.executor_proxy.registerExternalEventTarget(
                    "http://localhost:{}".format(self.server_port))
            except socket.error:
                sys.stdout.write(".")
            else:
                break

        print

        # Start dialogue manager
        self.dialogue_manager = BarebonesDialogueManager(self, self.executor_proxy)

        self.user_name = "User"

        # Wait for executor to fully boot
        self.append_log(">> Please wait, initializing...")
        while not self.executor_ready_flag.wait(0.1):
            time.sleep(.1)
        self.append_log(">> Ready!")
        self.append_log("  ")

        # Tell the user we are ready
        self.append_log("Hello.", "System")

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

    def append_log(self, message, agent=None):
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
                reply = RESPONSE_ERROR
            except IOError:
                self.append_log("Could not connect to NLPipeline.")
                raise
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

        # Initiate a specCompiler to hang around and give us immediate parser feedback
        self.compiler = SpecCompiler()
        self.compiler.proj = self.ltlmop.proj

    def clear(self):
        self.spec = []
        self.ltlmop.append_log("Cleared the specification.", "System")

    def execute(self):
        # TODO: don't resynthesize if the specification hasn't changed?
        #       i.e. distinguish between resuming from pause, versus a new command

        if not self.spec:
            return "You haven't given me any orders I understand."

        # pause
        self.executor.pause()

        self.ltlmop.append_log("Please wait...", "System")

        # trigger resynthesis
        success = self.executor.resynthesizeFromNewSpecification(self.get_spec())
        if success:
            # resume
            self.executor.resume()
            return "Got it. I'm carrying out your orders."
        else:
            self.clear()
            return ("Sorry, I can't come up with a plan that will carry out all your orders. "
                    "Try giving fewer commands at a time.")

    def tell(self, message):
        """ take in a message from the user, return a response.
            WARNING: this is effectively running in non-main thread"""
        msg = message.lower().strip()
        if msg == "clear":
            self.clear()
            return
        elif msg == "go":
            return self.execute()
        elif msg == "wait":
            self.executor.pause()
            return "Paused."
        elif msg == "status":
            if not self.executor.isRunning():
                return "Currently paused."

            curr_goal_num = self.executor.getCurrentGoalNumber()
            if curr_goal_num is None:
                return "I'm not doing anything right now."
            else:
                return self.executor.getCurrentGoalDescription()
        elif msg == "list":
            return "\n".join(self.spec)
        else:
            # Ask parser if this individual line is OK
            # FIXME: Because _writeLTLFile() is so monolithic, this will
            # clobber the `.ltl` file
            # FIXME: This may only work with SLURP
            self.compiler.proj.specText = message.strip()
            spec, traceback_tree, response = self.compiler._writeLTLFile()
            if spec is not None:
                self.spec.append(message.strip())
            return response[0]

    def get_spec(self):
        """ return the current specification as one big string """
        return "\n".join(self.base_spec + self.spec)

