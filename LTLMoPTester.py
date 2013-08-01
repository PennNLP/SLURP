#!/usr/bin/env python

import os
import sys
import getpass
import socket
import multiprocessing
import threading
import logging
import time
import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer

import project
from specCompiler import SpecCompiler
import execute


# TODO: Refactor Config class into global variables
class Config:
    # base_spec_file = os.path.join(ltlmop_root, "src", "examples", "gumbotest", "skeleton.spec")
    base_spec_file = os.path.join("firefighting", "firefighting.spec")
    executor_listen_port = 20000
    ltlmop_listen_port = 20001


class LTLMoPTester(object):

    def __init__(self):
        self.map_bitmap = None
        self.robot_pos = None
        self.fiducial_positions = {}
        self.proj = project.Project()
        self.proj.loadProject(Config.base_spec_file)

        # Start execution context
        print "Starting executor..."
        self.executor_ready_flag = threading.Event()
        self.executor_process = multiprocessing.Process(
            target=execute.execute_main,
            args=(Config.executor_listen_port,
                  self.proj.getFilenamePrefix() + ".spec",
                  None, False))
        self.executor_process.start()

        # Start our own xml-rpc server to receive events from execute
        self.xmlrpc_server = SimpleXMLRPCServer(("localhost", Config.ltlmop_listen_port),
                                                logRequests=False, allow_none=True)

        # Register functions with the XML-RPC server
        self.xmlrpc_server.register_function(self.handle_event)

        # Kick off the XML-RPC server thread
        self.xmlrpc_server_thread = threading.Thread(target=self.xmlrpc_server.serve_forever)
        self.xmlrpc_server_thread.daemon = True
        self.xmlrpc_server_thread.start()
        print "LTLMoPTester listening for XML-RPC calls on \
               http://localhost:{} ...".format(Config.ltlmop_listen_port)

        # Connect to executor
        print "Connecting to executor...",
        while True:
            try:
                self.executor_proxy = xmlrpclib.ServerProxy(
                    "http://localhost:{}".format(Config.executor_listen_port),
                    allow_none=True)

                # Register with executor for event callbacks
                self.executor_proxy.registerExternalEventTarget(
                    "http://localhost:{}".format(Config.ltlmop_listen_port))
            except socket.error:
                sys.stdout.write(".")
            else:
                break

        print

        # Start dialogue manager
        self.dialogue_manager = BarebonesDialogueManager(self, self.executor_proxy)

        # Load in robot icon
        # https://commons.wikimedia.org/wiki/File:Hamton_the_hamster.gif
        # Figure out the user's name, if we can
        try:
            self.user_name = getpass.getuser().title()
        except:
            self.user_name = "User"

        # Wait for executor to fully boot
        self.append_log(">> Please wait, initializing...")
        while not self.executor_ready_flag.wait(0.1):
            time.sleep(.1)
        self.append_log(">> Ready!")
        self.append_log("  ")

        # Tell the user we are ready
        self.append_log("Hello.", "System")

    def handle_event(self, event_type, event_data):
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

    def on_close(self, event):
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
        event.Skip()

    def append_log(self, message, agent=None):
        if agent:
            print agent + " : " + message
        else:
            print message

    def get_input(self):  # wxGlade: LTLMoPTester.<event_handler>
        user_text = raw_input('> ')
        if user_text == "":
            return
        # echo
        self.append_log(user_text, self.user_name)
        # response
        if self.dialogue_manager is None:
            self.append_log("Dialogue manager not initialized", "!!! Error")
        else:
            self.on_receive_reply(self.dialogue_manager.tell(user_text))

    def on_receive_reply(self, result):
        """ when the dialoguemanager has gotten back to us """
        try:
            result = result.get()
        except IOError:
            logging.exception("Could not connect to SLURP pipeline.")
            result = None

        if result:
            self.append_log(result, "System")

    def on_clear_commands(self, event):
        self.dialogue_manager.clear()
        event.Skip()

    def on_execute(self, event):
        self.dialogue_manager.execute()
        event.Skip()

# end of class LTLMoPTester


class BarebonesDialogueManager(object):
    def __init__(self, ltlmoptester, executor, base_spec=None):
        """ take reference to execution context and gui_window
            optionally initialize with some base spec text """

        self.ltlmop = ltlmoptester
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

        # pause
        self.executor.pause()

        self.ltlmop.append_log("Please wait...", "System")

        # trigger resynthesis
        success = self.executor.resynthesizeFromNewSpecification(self.get_spec())
        if success:
            # resume
            self.executor.resume()
            self.ltlmop.append_log("Doing as you asked.", "System")
        else:
            self.ltlmop.append_log(
                "I'm sorry, I can't do that.  Please try something else.", "System")

    def tell(self, message):
        """ take in a message from the user, return a response.
            WARNING: this is effectively running in non-main thread"""
        msg = message.lower().strip()
        if msg == "clear":
            self.clear()
            return
        elif msg == "go":
            self.execute()
            return
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


if __name__ == "__main__":
    LTLMOPTESTER = LTLMoPTester()
    while True:
        LTLMOPTESTER.get_input()
