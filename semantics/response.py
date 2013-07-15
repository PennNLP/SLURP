#!/usr/bin/env python
"""Generates a response"""

from semantics.lexical_constants import ACTION_ALIASES

# Semantics constants
KNOWN_ACTIONS = set(ACTION_ALIASES.values())

# Response constants
DUNNO = "Sorry, I don't know how to%s."
GOTIT = "Got it. I'll%s."
MISUNDERSTAND = "Sorry, I didn't understand that at all."

def make_response(new_commands, kb_response):
    """Make a response based on the new commands
    and the knowledge base response."""
    if not new_commands:
        # Use knowledge base response if available, otherwise give up
        if kb_response:
            return kb_response
        else:
            return MISUNDERSTAND

    # Split into good and bad commands, futher filtering the good ones
    good_commands = []
    bad_commands = []
    # TODO: handle unknown entities

    for c in new_commands:
        if c.action in KNOWN_ACTIONS:
            good_commands.append(c)
        else:
            bad_commands.append(c)

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
        actions = commands[0].readable()
    elif len(commands) == 2:
        actions = " and ".join(command.readable() for command in commands)
    else:
        actions = ", ".join(command.readable() for command in commands[:-1])
        actions += ", and " + commands[-1].readable()

    return actions
