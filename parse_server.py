#!/usr/bin/env python
"""Demonstration of the Penn NLP pipeline and semantics processing."""

import sys
import json
import socket
from socket import timeout
import threading

from commproxy import _parse_msg
from pennpipeline import PennPipeline
from semantics import knowledge, tree
from semantics.knowledge import (SEARCH_ACTION, GO_ACTION, GET_ACTION, FOLLOW_ACTION,
                                 SEE_ACTION, TELL_ACTION, ACTION_ALIASES)


SECRET_CODE = ",oO-i2De<2W5NVuJa6E"
_WORLD_KNOWLEDGE = None

SEARCH_PROP = "search"
FOLLOW_PROP = "follow_me"
EXPLORE_PROP = "explore"
DEFUSE_PROP = "defuse"
COMPLEX_ACTUATORS = {"get": "get_obj"}
SIMPLE_ACTUATORS = set((SEARCH_PROP, FOLLOW_PROP, DEFUSE_PROP, EXPLORE_PROP))

# Semantics constants
KNOWN_ACTIONS = set((SEARCH_ACTION, GO_ACTION, GET_ACTION, FOLLOW_ACTION, SEE_ACTION,
                     TELL_ACTION))
THEME = "Theme"
LOCATION = "Location"
PATIENT = "Patient"
SOURCE = "Source"
UNDERSPECIFIED = "*"

# Response constants
OKAY = "Understood. I'm carrying out your orders now."
FAILURE = "Sorry, I can't make a plan from those orders. Here's why: "
DUNNO = "Sorry, I don't know how to %s."
GOTIT = "Got it. I'll %s."
MISUNDERSTAND = "Sorry, I didn't understand that at all."


class ServiceSocket:
    """Listen on a socket for messages and dispatch them appropriately."""
    name = "ServiceSocket"

    def __init__(self, port, msg_sep="\n"):
        self.msg_sep = msg_sep
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('', port))
        self._sock.listen(1)

        # Initialize pipeline
        self._pipeline = PennPipeline()

        # Make a thread that keeps the connection alive
        accept_thread = threading.Thread(target=self._accept)
        accept_thread.name = "Listener"
        accept_thread.daemon = True
        print "Starting listening thread on port %d..." % port
        accept_thread.start()

    def _accept(self):
        """Accept connections on the listening port."""
        print "Listening for connections..."
        while True:
            try:
                conn, addr = self._sock.accept()
                print "%s: Connected to %s" % (self.name, str(addr))

                # Start up a new thread to handle the client
                name = "Client " + str(addr)
                accept_thread = threading.Thread(target=self._handle_client, args=(conn, name))
                accept_thread.name = name
                accept_thread.daemon = True
                print "%s: Starting thread for client %s" % (self.name, accept_thread.name)
                accept_thread.start()
            except timeout:
                continue
            # pylint: disable=W0702
            except:  # Because any error can occur here during shutdown
                pass

    def _handle_client(self, conn, name):
        """Process requests from a client."""
        send_error = False
        while True:
            try:
                print "Waiting for data..."
                buff = conn.recv(4096)
            except timeout:
                continue
            # pylint: disable=W0702
            except:
                # Break the connection for all more serious errors
                break
            if not buff:
                break
            while buff:
                if self.msg_sep:
                    msg, buff = _parse_msg(buff, self.msg_sep)
                else:
                    msg, buff = buff, None
                if msg:
                    response = self._process_text(msg)
                    if response:
                        print "Sending:", response
                        try:
                            conn.sendall(json.dumps(response))
                        except:
                            # Give up on the client entirely
                            print "%s: Error sending message." % self.name
                            send_error = True
                            break
                else:
                    print "%s: Received an incomplete message: %s" % (self.name, repr(buff))
                    break

            # Break out if there was a sending error
            if send_error:
                break

        print "%s: %s disconnected." % (self.name, name)

    def _process_text(self, msg):
        """Run a string through the NLP pipeline."""
        # Remove backlashes since they are sometimes in the input for no good reason
        text = msg.strip().replace('\\', '')

        if text.startswith(SECRET_CODE):
            knowledge_demo = True
            text = text[len(SECRET_CODE):]
        else:
            knowledge_demo = False

        # Return nothing if there was not text
        if not text:
            return {}

        if knowledge_demo:
            print "Secret demo mode!"
            # pylint: disable=W0603
            global _WORLD_KNOWLEDGE
            if not _WORLD_KNOWLEDGE or text == "reset":
                _WORLD_KNOWLEDGE = knowledge.Knowledge()

            world_knowledge = _WORLD_KNOWLEDGE
        else:
            world_knowledge = knowledge.Knowledge()

        response = {}

        # Run the parse pipeline
        parse = self._pipeline.parse_text(text)
        response['parse'] = parse.replace('(', '[').replace(')', ']')

        # Get the results from semantics
        results = world_knowledge.process_parse_tree(parse, text)
        answer, frame_trees, new_commands = results[0], results[1], results[3]

        # Use the answer if there was one and it was a string
        user_response = answer if answer and isinstance(answer, str) else ""

        # Also, echo back any new commands
        command_echo = make_response(new_commands)

        # Combine the responses as needed.
        if not user_response:
            user_response = command_echo
        elif new_commands:
            user_response += " " + command_echo

        response['response'] = user_response

        if frame_trees is not None:
            modified_trees = [str(modified_parse_tree[1]).replace('(', '[').replace(')', ']')
                              for modified_parse_tree in frame_trees
                              if (len(modified_parse_tree) > 1 and
                                  isinstance(modified_parse_tree[1], tree.Tree))]
            response['trees'] = list(set(modified_trees))

            frames = [frame_dict for frame_dict in [frame[0] for frame in frame_trees
                                                    if isinstance(frame[0], dict)]]
            response['frames'] = frames
        else:
            response['trees'] = []
            response['frames'] = []

        # Extract the command queue and add it to the response.
        # This is turned off for now because better talkback makes it
        # unnecessary.
        # command_queue = world_knowledge.command_queue
        # if command_queue:
        #    response['response'] += " Commands: " + str(command_queue)

        return response

    def shutdown(self):
        """Shut down the listening socket."""
        print "%s: Shutting down socket." % self.name
        self._sock.shutdown(socket.SHUT_RDWR)
        self._sock.close()
        print "%s: Socket closed." % self.name


def main(port):
    """Start listening for input."""

    # Set up listener
    listener = ServiceSocket(port)

    # Finally, wait for input before quitting
    try:
        while True:
            text = raw_input("").strip()
            if text == "q":
                break
    except (KeyboardInterrupt, EOFError):
        pass
    except:
        raise
    finally:
        listener.shutdown()


def make_response(new_commands):
    """Make a response based on the new commands."""
    # Give up if there were no new commands
    if not new_commands:
        return MISUNDERSTAND

    # Split into good and bad commands, futher filtering the good ones
    good_commands = []
    bad_commands = []
    for verb, target in new_commands:
        # Skip the SEE_ACTION entirely
        if verb == SEE_ACTION or (verb in ACTION_ALIASES and ACTION_ALIASES[verb] == SEE_ACTION):
            continue

        # Filter to known verbs, aliasing if needed
        if verb in ACTION_ALIASES or verb in SIMPLE_ACTUATORS or verb in COMPLEX_ACTUATORS:
            try:
                verb = ACTION_ALIASES[verb]
            except KeyError:
                pass
            good_commands.append((verb, target))
        else:
            bad_commands.append((verb, target))

    # Build up the response
    response = ""
    if good_commands:
        response += GOTIT % _join_commands(good_commands)

    if bad_commands:
        # Pad the initial response if there's something there
        if response:
            response += " "

        response += DUNNO % _join_commands(bad_commands)

    # Return the response made
    return response if response else MISUNDERSTAND


def _join_commands(commands):
    """Join the commands in a semi-grammatical fashion."""
    # Put in ands and commas as needed
    if len(commands) == 1:
        actions = _englishify_command(commands[0])
    elif len(commands) == 2:
        actions = " and ".join(_englishify_command(command) for command in commands)
    else:
        actions = ", ".join(_englishify_command(command) for command in commands[:-1])
        actions += ", and " + _englishify_command(commands[-1])

    return actions


def _englishify_command(command):
    """Turn a command into its reasonably grammatical equivalent."""
    verb, target = command

    # Check for known argument structures
    determiner = "the"
    preposition = None
    obj2 = None

    # Some specific rules for extracting targets
    if LOCATION in target:
        preposition = "to" if verb != SEARCH_PROP else None
        obj1 = target[LOCATION]
    elif PATIENT in target:
        obj1 = target[PATIENT]
    elif THEME in target:
        obj1 = target[THEME]
        if SOURCE in target:
            # Always gonna be retrieve X from Y
            obj2 = "from " + target[SOURCE]
    else:
        # If we're this lost, print a warning and grab whatever key we can
        print "Warning: Couldn't understand this target when building a response:", target
        try:
            obj1 = target.values()[0]
        except IndexError:
            obj1 = "unknown"

    # Convert the action if possible
    try:
        verb = ACTION_ALIASES[verb]
    except KeyError:
        if verb in SIMPLE_ACTUATORS or verb in COMPLEX_ACTUATORS:
            determiner = "all"
            # Pluralize the target the hackish way
            obj1 += "s"
        else:
            # Switch to indefinite on unknowns
            determiner = "a"

    # Build the response
    response = " ".join((verb, preposition, determiner, obj1)) if preposition else \
        " ".join((verb, determiner, obj1))

    if obj2:
        response += " " + obj2

    return response


if __name__ == "__main__":
    try:
        main(int(sys.argv[1]))
    except (IndexError, ValueError):
        print >> sys.stderr, "Usage parse_server port"
