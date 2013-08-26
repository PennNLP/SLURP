"""
Generates a logical specification from natural language.

"""

# Copyright (C) 2011-2013 Kenton Lee, Constantine Lignos, and Ian Perera
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import sys
import re
from collections import OrderedDict, defaultdict
from copy import deepcopy

from semantics.lexical_constants import (
    SEARCH_ACTION, GO_ACTION, FOLLOW_ACTION, SEE_ACTION, BEGIN_ACTION,
    AVOID_ACTION, PATROL_ACTION, CARRY_ACTION, STAY_ACTION, ACTIVATE_ACTION,
    DEACTIVATE_ACTION, UNDERSTOOD_SENSES)
from semantics.new_structures import Event, Assertion
from semantics.parsing import process_parse_tree
from pipelinehost import PipelineClient
from semantics.new_knowledge import KnowledgeBase
from ltlbroom.ltl import (
    env, and_, or_, sys_, not_, iff, next_, always, always_eventually, implies,
    space, mutex_, ALWAYS, EVENTUALLY, OR)


# Debug constants
COMMAND_DEBUG = False

# Semantics constants
LOCATION = "location"
UNDERSPECIFIED = "*"

# Generation constants
MEM = "mem"
DONE = "done"
VISIT = "visit"
REACT = "react"
DELIVER = "deliver"
FOLLOW_STATIONARY = "env_stationary"
ENV_STATIONARY = "TARGET_IS_STATIONARY"
FOLLOW_SENSORS = "FOLLOW_SENSOR_CONSTRAINTS"
STAY_THERE = "STAY_THERE"

# Actuators
SWEEP = "sweep"
PICKUP = "pickup"
DROP = "drop"
HOLDING = "holding"

# Talkback constants
GOTIT = "Got it. I can {!r}."
MISUNDERSTAND = "Sorry, I didn't understand that at all."


class SpecChunk(object):
    """Class for holding system or environment lines for a specification."""
    SYS = "System"
    ENV = "Environment"
    VALID_TYPES = (SYS, ENV)

    def __init__(self, explanation, lines, line_type, command):
        if not isinstance(explanation, basestring):
            raise ValueError("Explanation must be a string/Unicode.")

        if not hasattr(lines, '__iter__'):
            raise ValueError("Lines must be an iterable.")

        if line_type not in SpecChunk.VALID_TYPES:
            raise ValueError("Spec line_type must be one of {}".format(SpecChunk.VALID_TYPES))

        self.explanation = explanation
        self.lines = lines
        self.type = line_type
        self.command = command

        # To be set later by consumer
        self.input = None
        self.goal_indices = set()
        self.highlights = [False for _ in range(len(self.lines))]

    def __repr__(self):
        return '<{} lines for {!r}: {}, Command: {}, Goals: {}>'.format(
            self.type, self.explanation, self.lines, _format_command(self.command),
            list(self.goal_indices))

    def __str__(self):
        return '<{} lines for {!r}: {}>'.format(self.type, self.explanation, self.lines)

    def contains_goal(self, goal_index):
        """Return whether this contains a goal with the specified index."""
        return goal_index in self.goal_indices

    def contains_line(self, line):
        """Return whether this contains a given line."""
        return line in self.lines

    def issys(self):
        """Returns whether this contains system lines."""
        return self.type == SpecChunk.SYS

    def isenv(self):
        """Returns whether this contains env lines."""
        return self.type == SpecChunk.ENV


class SpecGenerator(object):
    """Enables specification generation using natural language."""

    def __init__(self):
        # Handlers
        self.GOALS = {PATROL_ACTION: self._gen_patrol, GO_ACTION: self._gen_go,
                      AVOID_ACTION: self._gen_avoid, SEARCH_ACTION: self._gen_search,
                      BEGIN_ACTION: self._gen_begin, FOLLOW_ACTION: self._gen_follow,
                      STAY_ACTION: self._gen_stay, CARRY_ACTION: self._gen_carry,
                      ACTIVATE_ACTION: self._gen_activate, DEACTIVATE_ACTION: self._gen_deactivate}

        self.REACTIONS = {GO_ACTION: _frag_react_go, STAY_ACTION: self._frag_stay,
                          AVOID_ACTION: _frag_react_avoid,
                          ACTIVATE_ACTION: _frag_react_activate,
                          DEACTIVATE_ACTION: _frag_react_deactivate}

        # TODO: Make this unnecessary
        self.NEG_REACTIONS = {ACTIVATE_ACTION: _frag_react_deactivate,
                              DEACTIVATE_ACTION: _frag_react_activate}

        # Information about the scenario will be updated at generation time
        self.sensors = None
        self.regions = None
        self.props = None
        self.react_props = None
        self.tag_dict = None

        # Knowledge base
        self.kbase = KnowledgeBase()

    def generate(self, text, sensors, regions, props, tag_dict, realizable_reactions=True,
                 verbose=True):
        """Generate a logical specification from natural language and propositions."""
        # Clean unicode out of everything
        text = text.encode('ascii', 'ignore')
        self.sensors = [astr.encode('ascii', 'ignore') for astr in sensors]
        self.regions = [astr.encode('ascii', 'ignore') for astr in regions]
        self.props = [astr.encode('ascii', 'ignore') for astr in props]
        self.tag_dict = {key.encode('ascii', 'ignore'):
                         [value.encode('ascii', 'ignore') for value in values]
                         for key, values in tag_dict.items()}

        if verbose:
            print "NL->LTL Generation called on:"
            print "Sensors:", self.sensors
            print "Props:", self.props
            print "Regions:", self.regions
            print "Tag dict:", self.tag_dict
            print "Text:", repr(text)
            print

        # Make lists for POS conversions, including the metapar keywords
        force_nouns = list(self.regions) + list(self.sensors)
        force_verbs = self.GOALS.keys()

        # Set up
        parse_client = PipelineClient()
        results = []
        responses = []
        custom_props = set()
        self.react_props = set()  # TODO: Make this a local
        custom_sensors = set()
        generation_trees = OrderedDict()

        # Add the actuator mutex
        if len(self.props) > 1:
            actuator_mutex = mutex_([sys_(prop) for prop in self.props], True)
            generation_trees["Safety assumptions"] = \
                {"Safety assumptions":
                 [SpecChunk("Robot can perform only one action at a time.",
                            [actuator_mutex, always(actuator_mutex)],
                            SpecChunk.SYS, None)]}

        for line in text.split('\n'):
            # Strip the text before using it and ignore any comments
            line = line.strip()
            line = _remove_comments(line)

            if not line:
                # Blank lines are counted as being processed correctly but are skipped
                results.append(True)
                responses.append('')
                continue

            # Init the generation tree to the empty result
            generated_lines = OrderedDict()
            generation_trees[line] = generated_lines

            if verbose:
                print "Sending to remote parser:", repr(line)
            parse = parse_client.parse(line, force_nouns, force_verbs=force_verbs)
            if verbose:
                print "Response from parser:", repr(parse)
            frames, new_commands, kb_response = \
                process_parse_tree(parse, line, self.kbase, quiet=True)

            frames, new_commands, kb_response = process_parse_tree(parse, line, self.kbase,
                                                                   quiet=not verbose)
            # Build the metapars
            # For now, assume success if there were commands or a kb_response
            success = bool(new_commands) or bool(kb_response)
            command_responses = [kb_response] if kb_response else []
            for command in new_commands:
                if COMMAND_DEBUG:
                    print "Processing command:"
                    print command
                try:
                    new_sys_lines, new_env_lines, new_custom_props, new_custom_sensors = \
                        self._apply_metapar(command)
                except KeyError as err:
                    cause = err.message
                    problem = \
                        "Could not understand {!r} due to error {}.".format(command.action, cause)
                    if verbose:
                        print >> sys.stderr, "Error: " + problem
                    command_responses.append(cause)
                    success = False
                    continue
                else:
                    command_responses.append(respond_okay(command.action))

                # Add in the new lines
                command_key = _format_command(command)
                if command_key not in generated_lines:
                    generated_lines[command_key] = []
                generated_lines[command_key].extend(new_sys_lines)
                generated_lines[command_key].extend(new_env_lines)

                # Add custom props/sensors
                custom_props.update(new_custom_props)
                custom_sensors.update(new_custom_sensors)

            # If we've got no responses, say we didn't understand at all.
            if not command_responses:
                command_responses.append(respond_nocommand())

            # Add responses and successes
            results.append(success)
            responses.append(' '.join(command_responses))
            # Add some space between commands
            if verbose:
                print

        if COMMAND_DEBUG:
            print "Generation trees:"
            for line, output in generation_trees.items():
                print line
                print output
                print

        # We need to modify non-reaction goals to be or'd with the reactions
        if realizable_reactions and self.react_props:
            # Dedupe and make an or over all the reaction properties
            reaction_or_frag = or_([sys_(prop) for prop in self.react_props])
            # HACK: Rewrite all the goals!
            # TODO: Test again with reaction propositions other than defuse
            for command_spec_chunks in generation_trees.values():
                for spec_chunks in command_spec_chunks.values():
                    for spec_chunk in spec_chunks:
                        if not spec_chunk.issys():
                            continue
                        spec_chunk.lines = [_insert_or_before_goal(reaction_or_frag, line)
                                            for line in spec_chunk.lines]

        # Aggregate all the propositions
        # Identify goal numbers as we loop over sys lines
        sys_lines = []
        # At the moment, there are no useless goals in specs, so we
        # begin at 0
        goal_idx = 0
        for input_text, command_spec_lines in generation_trees.items():
            for command, spec_lines_list in command_spec_lines.items():
                for spec_lines in spec_lines_list:
                    if not spec_lines.issys():
                        continue
                    for line in spec_lines.lines:
                        spec_lines.input = input_text
                        sys_lines.append(line)
                        if isgoal(line):
                            spec_lines.goal_indices.add(goal_idx)
                            goal_idx += 1

        # Filter out any duplicates from the env_lines
        env_lines = OrderedDict()
        for command_spec_lines in generation_trees.values():
            for spec_lines_list in command_spec_lines.values():
                for spec_lines in spec_lines_list:
                    if not spec_lines.isenv():
                        continue
                    for line in spec_lines.lines:
                        env_lines[line] = None
        env_lines = env_lines.keys()

        # Convert sets to lists for the caller
        custom_props = list(custom_props)
        custom_sensors = list(custom_sensors)

        if verbose:
            print "Spec generation complete."
            print "Results:", results
            print "Responses:", responses
            print "Environment lines:", env_lines
            print "System lines:", sys_lines
            print "Custom props:", custom_props
            print "Custom sensors:", custom_sensors
            print "Generation trees:", generation_trees

        return (env_lines, sys_lines, custom_props, custom_sensors, results, responses,
                generation_trees)

    def _apply_metapar(self, command):
        """Generate a metapar for a command."""
        # Patch up intransitives as activate if needed
        # TODO: This has only been tested with defuse and may not work for other actions.
        if (command.action not in self.GOALS and
            command.action in UNDERSTOOD_SENSES):
            print "Changed action {} to an activate command.".format(command.action)
            if command.theme:
                command.theme.name = command.action
            command.action = "activate"

        try:
            handler = self.GOALS[command.action]
        except KeyError:
            raise KeyError('Unknown action {0}.'.format(command.action))

        if command.condition:
            return self._gen_conditional(command)
        else:
            # Extract the targets from args and pass them as arguments
            return handler(command)

    def _expand_argument(self, argument, command):
        """Return a list of the arguments created by expanding the argument if its quantified."""
        # What to return if quantification fails
        default_return = [argument]
        try:
            quant = argument.quantifier.type
        except AttributeError:
            return default_return
        if quant == "all" or (command.negation and quant == "any"):
            # TODO: Handle more than one tag
            try:
                tag = argument.description[0]
            except (IndexError, TypeError):
                print >> sys.stderr, "Error: Could not get description of {}.".format(argument)
                return default_return

            try:
                members = sorted(self.tag_dict[tag])
            except KeyError:
                print "Error: Could not get members of tag {!r}.".format(argument.description)
                return default_return

            # Unroll into copies of the command.
            new_args = [deepcopy(argument) for _ in range(len(members))]
            for new_arg, member in zip(new_args, members):
                new_arg.quantifier.type = "exact"
                new_arg.quantifier.number = 1
                new_arg.name = member

            return new_args
        else:
            return default_return

    # The return signature of the statement generators is:
    # ([system lines], [env lines], [custom propositions], [custom sensors])
    def _gen_follow(self, command):
        """Generate statements for following."""
        # Env is stationary iff in the last state change our region and the env's region were stable
        stationary_explanation = "Definition of when the target is moving."
        stationary_safeties = \
            always(iff(next_(sys_(FOLLOW_STATIONARY)),
                       ENV_STATIONARY))
        stationary_lines = SpecChunk(stationary_explanation, [stationary_safeties], SpecChunk.SYS,
                                     command)

        # Stay there if environment is changing
        stay_there_explanation = "React immediately to the target moving."
        stay_there_safeties = \
            always(implies(not_(next_(sys_(FOLLOW_STATIONARY))), self._frag_stay()))
        stay_there_lines = \
            SpecChunk(stay_there_explanation, [stay_there_safeties], SpecChunk.SYS, command)

        # Match the sensor location to ours
        follow_goals = \
            [SpecChunk("Follow the target to {!r}.".format(region),
             [always_eventually(
              implies(and_((sys_(FOLLOW_STATIONARY), env(region))), sys_(region)))],
             SpecChunk.SYS, command) for region in self.regions]
        follow_env = SpecChunk("Target must obey map topology.",
                               [FOLLOW_SENSORS], SpecChunk.ENV, command)
        return ([stationary_lines, stay_there_lines] + follow_goals, [follow_env],
                [FOLLOW_STATIONARY], [])

    def _gen_conditional(self, command, assume_eventual_relief=False):
        """Generate a conditional action"""
        # TODO: Properly document and condition assume_eventual_relief

        env_chunks = []
        if isinstance(command.condition, Event):
            # Validate the condition
            if not command.condition.theme:
                raise KeyError("Cannot understand condition:\n{}".format(command.condition))
            condition = command.condition.theme.name
            if condition not in self.sensors:
                raise KeyError(
                    "No sensor to detect condition {!r}".format(command.condition.theme.name))
            if command.condition.sensor != SEE_ACTION:
                raise KeyError(
                    "Cannot use action {!r} as a condition".format(command.condition.action))
            condition_frag = env(condition)
            explanation = "To react to {!r},".format(condition)
            if assume_eventual_relief:
                relief_explanation = "Assume {!r} eventually goes away.".format(condition)
                relief = always_eventually(not_(condition_frag))
                relief_chunk = SpecChunk(relief_explanation, [relief], SpecChunk.ENV, command)
                env_chunks.append(relief_chunk)
        elif isinstance(command.condition, Assertion):
            # TODO: Add support for assertions not about "you". Ex: If there is a hostage...
            # Validate the condition
            if not command.condition.location:
                raise KeyError("Cannot understand condition:\n{}".format(command.condition))
            condition = command.condition.location.name
            condition_frag = sys_(condition)
            explanation = "When in {!r},".format(condition)
        else:
            raise KeyError("Cannot understand condition:\n{}".format(command.condition))

        # Validate the action
        action = command.action
        if action not in self.REACTIONS:
            raise KeyError("Unknown reaction {!r}".format(action))

        # Create the right type of reaction
        new_props = []
        if action in self.props:
            # Simple actuator
            reaction_prop = action
        else:
            # Reaction proposition
            reaction_prop_name = REACT + "_" + condition
            reaction_prop = sys_(reaction_prop_name)
            new_props.append(reaction_prop_name)
            self.react_props.add(reaction_prop_name)

        # Generate the response
        sys_statements = []
        if action in (GO_ACTION, AVOID_ACTION):
            # Go is unusual because the outcome is not immediately satisfiable
            if not command.location:
                raise KeyError("No location in go reaction")
            destination = command.location.name

            # Negation is easy, so we take a shortcut
            if ((action == GO_ACTION and command.negation) or
                    (action == AVOID_ACTION and not command.negation)):
                sys_statements.append(always(implies(next_(condition_frag),
                                                     not_(next_(sys_(destination))))))
                explanation += " avoid {!r}.".format(command.location.name)
            else:
                destination_stmt = sys_(destination)
                # New goal for where we should go
                go_goal = always_eventually(implies(reaction_prop, destination_stmt))
                # Safety that persists
                go_safety = \
                    always(iff(next_(reaction_prop),
                               or_([reaction_prop, next_(condition_frag)])))
                # Make sure we act immediately: []((!react & next(react) )-> stay_there)
                stay_there = always(implies(and_((not_(reaction_prop), next_(reaction_prop))),
                                            self._frag_stay()))

                sys_statements.extend([go_goal, go_safety, stay_there])
                explanation += " go to {!r}.".format(command.location.name)
        elif action == STAY_ACTION:
            sys_statements.append(always(iff(next_(condition_frag), next_(reaction_prop))))
            sys_statements.append(always(implies(or_([reaction_prop, next_(reaction_prop)]),
                                                 STAY_THERE)))
            explanation += " stay there."
        else:
            if command.theme.name not in self.props:
                raise KeyError("Unknown actuator {!r}".format(command.theme.name))
            # Otherwise we are always creating reaction safety
            sys_statements.append(always(iff(next_(condition_frag), next_(reaction_prop))))
            template = " {} {!r}." if not command.negation else " do not {} {!r}."
            explanation += template.format(action, command.theme.name)

            if not command.negation:
                handler = self.REACTIONS[action]
            else:
                handler = self.NEG_REACTIONS[action]
            reaction_frag = handler(command)
            react = always(implies(next_(reaction_prop), reaction_frag))
            stay_there = always(implies(or_([reaction_prop, next_(reaction_prop)]),
                                       self._frag_stay()))
            sys_statements.extend([react, stay_there])

        sys_chunk = SpecChunk(explanation, sys_statements, SpecChunk.SYS, command)
        return ([sys_chunk], env_chunks, new_props, [])

    def _gen_stay(self, command):
        """Generate statements to stay exactly where you are."""
        sys_lines = SpecChunk("Stay in the same place.", [always_eventually(self._frag_stay())],
                              SpecChunk.SYS, command)
        return ([sys_lines], [], [], [])

    def _frag_stay(self, command=None):  # pylint: disable=W0613
        """Generate fragments to reactively go somewhere."""
        return STAY_THERE

    def _gen_carry(self, command):
        """Generate statements for carrying items from one region to another."""
        if not all((command.theme, command.source, command.destination)):
            raise KeyError("Missing item, source, or destination for carry.")

        item = command.theme.name
        source = command.source.name
        dest_arg = command.destination
        destinations = [dest.name for dest in self._expand_argument(dest_arg, command)]

        # Start the carry actuators and props as off
        start_explanation = "Nothing is carried or delivered at the start."
        deliver_mems = [_prop_mem(dest, DELIVER) for dest in destinations]
        holding_props = [HOLDING]
        start_lines = [_frag_props_off([PICKUP, DROP] + deliver_mems + holding_props)]
        start_chunk = SpecChunk(start_explanation, start_lines, SpecChunk.SYS, command)

        pickup_explanation = "Only pick up if you can carry more."
        pickup_lines = [always(implies(next_(sys_(HOLDING)), not_(next_(sys_(PICKUP)))))]
        pickup_chunk = SpecChunk(pickup_explanation, pickup_lines, SpecChunk.SYS, command)

        drop_explanation = "Only drop if you are carrying something."
        drop_lines = [always(implies(not_(next_(sys_(HOLDING))), not_(next_(sys_(DROP)))))]
        drop_chunk = SpecChunk(drop_explanation, drop_lines, SpecChunk.SYS, command)

        stay_explanation = "Stay where you are when picking up and dropping."
        stay_lines = [always(implies(or_([sys_(PICKUP), sys_(DROP)]),
                                     self._frag_stay()))]
        stay_chunk = SpecChunk(stay_explanation, stay_lines, SpecChunk.SYS, command)

        source_explanation = "Pick up {!r} in {!r}.".format(item, source)
        source_lines = [always(iff(and_([or_([and_([sys_(source), sys_(PICKUP)]), sys_(HOLDING)]),
                                         not_(sys_(DROP))]),
                                   next_(sys_(HOLDING))))]
        source_chunk = SpecChunk(source_explanation, source_lines, SpecChunk.SYS, command)

        # This is not a list comprehension solely for readability
        delivery_chunks = [_chunk_deliver(command, item, dest, mem_dest)
                           for dest, mem_dest in zip(destinations, deliver_mems)]

        return ([start_chunk, pickup_chunk, drop_chunk, stay_chunk, source_chunk] + delivery_chunks,
                [], deliver_mems + holding_props, [])

    def _gen_begin(self, command):
        """Generate statements to begin in a location."""
        region = command.theme.name
        explanation = "The robot begins in {!r}.".format(region)
        sys_lines = SpecChunk(explanation, [sys_(region)], SpecChunk.SYS, command)
        return ([sys_lines], [], [], [])

    def _gen_patrol(self, command):
        """Generate statements to always eventually be in a location."""
        regions = [location.name for location in self._expand_argument(command.location, command)]
        sys_chunks = []
        for region in regions:
            explanation = "Continuously visit {!r}.".format(region)
            sys_chunks.append(SpecChunk(explanation, [always_eventually(sys_(region))],
                                        SpecChunk.SYS, command))
        return (sys_chunks, [], [], [])

    def _gen_go(self, command):
        """Generate statements to go to a location once."""
        # Avoid if it's negated
        if command.negation:
            return self._gen_avoid(command)

        try:
            regions = [location.name for location in self._expand_argument(command.location, command)]
        except AttributeError:
            raise KeyError("Could not understand location for 'go' command.")
        # Raise an error if any of the regions are bad.
        for region in regions:
            if region not in self.regions:
                raise KeyError("Cannot go to location {!r} "
                               "because it is not on the map.".format(region))

        sys_chunks = []
        mem_props = []
        for region in regions:
            # Set memory false initially
            mem_prop = _prop_mem(region, VISIT)
            explanation1 = "Initially, {!r} has not been visited.".format(region)
            init_off = not_(sys_(mem_prop))
            init_chunk = SpecChunk(explanation1, [init_off], SpecChunk.SYS, command)
            sys_chunks.append(init_chunk)

            sys_lines = _frag_atleastonce(mem_prop, next_(sys_(region)))
            explanation2 = "Visit {!r}.".format(region)
            sys_chunk = SpecChunk(explanation2, sys_lines, SpecChunk.SYS, command)
            mem_props.append(mem_prop)
            sys_chunks.append(sys_chunk)

        return (sys_chunks, [], mem_props, [])

    def _gen_avoid(self, command):
        """Generate statements for never going to a location."""
        regions = [location.name for location in self._expand_argument(command.location, command)]
        spec_chunks = []
        for region in regions:
            explanation1 = "Do not go to {!r}.".format(region)
            sys_lines1 = SpecChunk(explanation1, [always(not_(sys_(region)))],
                                   SpecChunk.SYS, command)
            explanation2 = "The robot does not begin in {!r}.".format(region)
            sys_lines2 = SpecChunk(explanation2, [not_(sys_(region))], SpecChunk.SYS, command)
            spec_chunks.extend([sys_lines1, sys_lines2])

        return (spec_chunks, [], [], [])

    def _gen_search(self, command):
        """Generate statements for searching a region."""
        regions = [location.name for location in self._expand_argument(command.location, command)]
        spec_chunks = []
        mem_props = []
        for region in regions:
            # Set memory false initially
            mem_prop = _prop_mem(region, SWEEP)
            init_off = [not_(sys_(mem_prop))]
            explanation0 = "Initially, {!r} has not been searched.".format(region)
            spec_chunks.append(SpecChunk(explanation0, init_off, SpecChunk.SYS, command))

            explanation1 = "Complete a search in {!r}.".format(region)
            cic_frag, cic_env = _frag_complete_context(SWEEP, sys_(region))
            alo_sys = _frag_atleastonce(mem_prop, cic_frag)
            mem_props.append(mem_prop)
            spec_chunks.append(SpecChunk(explanation1, alo_sys, SpecChunk.SYS, command))

        explanation2 = "Assume that searches eventually complete."
        env_chunk = SpecChunk(explanation2, cic_env, SpecChunk.ENV, command)

        return (spec_chunks, [env_chunk], mem_props, [_prop_actuator_done(SWEEP)])

    def _gen_activate(self, command, negated=False):
        """Generate statements for activating an actuator."""
        try:
            actuator = command.theme.name
        except AttributeError:
            raise KeyError("Missing actuator for activate/deactivate.")

        # If negation isn't set, allow the command to set it
        negated = negated or command.negation
        actuator_frag = sys_(actuator) if not negated else not_(sys_(actuator))
        when = "Always" if not negated else "Never"

        chunks = []
        if command.location:
            regions = [location.name for location in
                       self._expand_argument(command.location, command)]
            for region in regions:
                # Generate a location-restricted action
                explanation = "{} activate {!r} in {!r}.".format(when, actuator, region)
                formula = always(implies(sys_(region), actuator_frag))
                chunks.append(SpecChunk(explanation, [formula], SpecChunk.SYS, command))
        else:
            # Always activate
            explanation = "{} activate {!r}.".format(when, actuator)
            formula = always(actuator_frag)
            chunks.append(SpecChunk(explanation, [formula], SpecChunk.SYS, command))

        return (chunks, [], [], [])

    def _gen_deactivate(self, command):
        """Generate statements for deactivating an actuator."""
        return self._gen_activate(command, True)


def _chunk_deliver(command, item, dest, mem_dest):
    """Return a chunk representing delivering an item to its destination."""
    delivery_explanation = "Deliver {!r} to {!r}.".format(item, dest)
    delivery_frag = and_([next_(sys_(dest)), next_(sys_(DROP))])
    delivery_chunk = SpecChunk(delivery_explanation, _frag_atleastonce(mem_dest, delivery_frag),
                               SpecChunk.SYS, command)
    return delivery_chunk


def _frag_atleastonce(mem_prop, fragment):
    """Generate fragments for performing an action at least once by using a memory proposition."""
    return [always(iff(next_(sys_(mem_prop)), or_((sys_(mem_prop), fragment)))),
            always_eventually(sys_(mem_prop))]


def _frag_complete_context(actuator, context_prop):
    """Generate fragments for completing an action in context."""
    actuator_done = _prop_actuator_done(actuator)
    eventually_actuator = always_eventually(env(actuator_done))
    return [and_((sys_(actuator), next_(env(actuator_done)), context_prop)), [eventually_actuator]]


def _frag_props_off(props):
    """Generate a fragment for props being off in the initial state."""
    return and_([not_(sys_(prop)) for prop in props])


def _frag_react_go(region):
    """Generate a fragment to reactively go somewhere."""
    return sys_(region)


def _frag_react_avoid(region):
    """Generate a fragment to reactively not go somewhere."""
    return not_(sys_(region))


def _frag_react_activate(command):
    """Generate a fragment to activate an actuator."""
    return next_(sys_(command.theme.name))


def _frag_react_deactivate(command):
    """Generate a fragment to activate an actuator."""
    return not_(next_(sys_(command.theme.name)))


def _prop_mem(region, event):
    """Generate a proposition for having visited a region."""
    return "_".join((MEM, event, region))


def _prop_actuator_done(actuator):
    """Generate a proposition for an actuator's completion."""
    return "_".join((actuator, DONE))


def _format_command(command):
    """Format a command nicely for display."""
    if not command:
        return "No command."

    action = ("Action: {!r}" if not command.negation else
              "Action: do not {!r}").format(command.action)
    fields = [action]
    if command.theme:
        fields.append("Argument: {!r}".format(command.theme.name))
    if command.location:
        fields.append("Location: {!r}".format(command.location.name))
    if command.source:
        fields.append("Source: {!r}".format(command.source.name))
    if command.destination:
        fields.append("Destination: {!r}".format(command.destination.name))

    return ", ".join(fields)


def _insert_or_before_goal(or_clause, statement):
    """Inserts a statement to be OR'd against what is already in the goal."""
    # Your hack is bad and you should feel bad.
    if ALWAYS + EVENTUALLY in statement and REACT not in statement:
        return statement.replace(ALWAYS + EVENTUALLY + '(',
                                 ALWAYS + EVENTUALLY + '(' + or_clause + space(OR))
    else:
        return statement


def _remove_comments(text):
    """Remove from the comment character to the end of the line."""
    return re.sub('#.*$', '', text).strip()


def goal_to_chunk(goal_idx, spec_chunks):
    """Return the unique SpecChunk that contain a goal index."""
    chunks = [spec_chunk for spec_chunk in spec_chunks if spec_chunk.contains_goal(goal_idx)]
    if len(chunks) == 1:
        return chunks[0]
    elif len(chunks) > 1:
        print >> sys.stderr, "Error: Found multiple chunks for goal {}.".format(goal_idx)
    # Zero length case returns None without printing an error
    return None


def line_to_chunks(line, spec_chunks):
    """Return all spec chunks that contain a line."""
    return [spec_chunk for spec_chunk in spec_chunks if line in spec_chunk.lines]


def chunks_from_gentree(gen_tree):
    """Return all the SpecChunk contained in a generation tree."""
    return [spec_lines for command_spec_lines in gen_tree.values()
            for spec_lines_list in command_spec_lines.values()
            for spec_lines in spec_lines_list]


def isgoal(line):
    """Return whether a line of LTL contains a goal."""
    return line.lstrip().startswith('(' + ALWAYS + EVENTUALLY)


def explain_conflict(conflicting_lines, gen_tree):
    """Explain the conflict between LTL statements."""
    chunks = chunks_from_gentree(gen_tree)
    conflicting_chunks = set.union(*[set(line_to_chunks(line, chunks))
                                     for line in conflicting_lines])

    # Mark the chunks for highlighting
    conflicting_lines_set = set(conflicting_lines)
    for chunk in conflicting_chunks:
        for idx, line in enumerate(chunk.lines):
            if line in conflicting_lines_set:
                chunk.highlights[idx] = True

    # Explain what's wrong with the goals
    goal_explanations = [(chunk.explanation, chunk.input) for chunk in conflicting_chunks
                         if any(isgoal(line) for line in chunk.lines)]
    n_conflicting_goals = len(goal_explanations)
    if n_conflicting_goals == 0:
        goal_problem = "No goals seem to be problematic."
    elif n_conflicting_goals == 1:
        goal_explanation, goal_input = goal_explanations[0]
        explain_template = ("The problematic goal comes from the statement {!r}." +
                            " The system cannot achieve the sub-goal {!r}.")
        goal_problem = explain_template.format(goal_input, goal_explanation)
    else:
        goal_template = "{!r} because of sub-goal {!r}"
        goal_problem = ("The problematic goals are:\n" +
                        "\n".join(goal_template.format(text, explanation)
                                  for explanation, text in goal_explanations))

    # Now explain the other issues
    other_explanations = [(chunk.explanation, chunk.input) for chunk in conflicting_chunks
                          if any(not isgoal(line) for line in chunk.lines)]
    other_problem = "The statements that cause the problem are:\n"
    # Group together sub-goals by the goal that generated them
    text_explains = defaultdict(list)
    for explanation, text in other_explanations:
        text_explains[text].append(explanation)
    other_template = "{!r} because of item(s): {}."
    other_problem += \
        "\n".join(other_template.format(text, ", ".join(repr(ex) for ex in explanations))
                  for text, explanations in sorted(text_explains.items()))

    explanation = goal_problem + "\n" + other_problem

    return explanation, gen_tree


def respond_nocommand():
    """Respond saying we did not understand at all."""
    return MISUNDERSTAND


def respond_okay(action):
    """Respond that we will perform the action."""
    return GOTIT.format(action)


def _test():
    """Test spec generation."""
    # pylint: disable=W0612
    specgen = SpecGenerator()
    env_lines, sys_lines, custom_props, custom_sensors, results, responses, gen_tree = \
        specgen.generate('\n'.join(sys.argv[1:]), ("bomb", "hostage", "badguy", "monkey"),
                         ("r1", "r2", "r3", "r4", "office", "classroom1", "classroom2"),
                         ("defuse", "pickup", "drop", "banana"),
                         {'odd': ['r1', 'r3']})
    for line in env_lines + sys_lines:
        print line


if __name__ == "__main__":
    _test()
