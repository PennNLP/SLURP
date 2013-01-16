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

from semantics.processing import process_parse_tree
from pipelinehost import PipelineClient
from semantics.new_knowledge import KnowledgeBase
from ltlbroom.ltl import env, and_, or_, sys_, not_, iff, next_, \
    always, always_eventually, implies, space, ALWAYS, EVENTUALLY, OR

# Debug constants
SEMANTICS_DEBUG = False

# Semantics constants
LOCATION = "location"
UNDERSPECIFIED = "*"

# Generation constants
MEM = "mem"
DONE = "done"
VISIT = "visit"
REACT = "react"
FOLLOW_STATIONARY = "env_stationary"
FOLLOW_SENSORS = "FOLLOW_SENSOR_CONSTRAINTS"

# Actuators
SWEEP = "sweep"


class SpecGenerator(object):
    """Enables specification generation using natural language."""

    def __init__(self):
        # Handlers
        self.GOALS = {'patrol': _gen_patrol, 'go': _gen_go,
                     'avoid': _gen_avoid, 'search': _gen_search,
                     'begin': _gen_begin, 'follow': self._gen_follow,
                     'stay': self._gen_stay}

        self.REACTIONS = {'go': _frag_react_go, 'stay': self._frag_stay}

        # Information about the scenario will be updated at generation time
        self.sensors = None
        self.regions = None
        self.props = None
        self.react_props = set()

        # Knowledge base
        self.kbase = KnowledgeBase()


    def generate(self, text, sensors, regions, props):
        """Generate a logical specification from natural language and propositions."""
        # Clean unicode out of everything
        text = text.encode('ascii', 'ignore')
        self.sensors = [astr.encode('ascii', 'ignore') for astr in sensors]
        self.regions = [astr.encode('ascii', 'ignore') for astr in regions]
        self.props = [astr.encode('ascii', 'ignore') for astr in props]

        print "NL->LTL Generation called on:"
        print "Sensors:", self.sensors
        print "Props:", self.props
        print "Regions:", self.regions
        print "Text:", repr(text)
        print

        # Make lists for POS conversions, including the metapar keywords
        force_nouns = list(self.regions) + list(self.sensors)
        force_verbs = list(self.props) + self.GOALS.keys()

        parse_client = PipelineClient()
        responses = []
        system_lines = []
        env_lines = []
        env_lines_set = set() # Used to track duplicates
        custom_props = set()
        custom_sensors = set()
        generation_trees = []
        for line in text.split('\n'):
            if not line:
                # Blank lines are counted as being processed correctly but are skipped
                responses.append(True)
                continue

            # Init the generation tree to the empty result
            generation_tree = [line.strip(), []]

            # Strip the text before using it and ignore any comments
            line = line.strip()
            line = _remove_comments(line)

            print "Sending to remote parser:", repr(line)
            parse = parse_client.parse(line, force_nouns, force_verbs=force_verbs)
            print "Response from parser:", parse
            user_response, semantics_result, semantics_response, new_commands = \
                process_parse_tree(parse, line)

            if SEMANTICS_DEBUG:
                print "Returned values from semantics:"
                print "User response:", repr(user_response)
                print "Semantics results:"
                for result in semantics_result:
                    print "\t" + str(result)
                print "Semantics response:", semantics_response
                print "New commands:", new_commands

            # Build the metapars
            failure = False
            for command in new_commands:
                try:
                    new_sys_lines, new_env_lines, new_custom_props, new_custom_sensors = \
                        self._apply_metapar(command)
                except KeyError as err:
                    print "Could not understand command {0} due to error {1}".format(command, err)
                    failure = True
                    continue

                system_lines.extend(new_sys_lines)
                # We need to ensure no duplicates are inserted into env_lines, so we keep
                # an redundant set. If we were requiring 2.7 we would use OrderedDict.
                for env_line in new_env_lines:
                    if env_line not in env_lines_set:
                        env_lines.append(env_line)
                        env_lines_set.add(env_line)
                custom_props.update(new_custom_props)
                custom_sensors.update(new_custom_sensors)
                # Add the statements as the children of the generation tree
                generation_tree[1].append([format_command(command), [new_env_lines, new_sys_lines]])

            # Add a true response if there were commands and no failures
            responses.append(new_commands and not failure)

            # Add this line's stuff to the generation tree
            generation_trees.append(generation_tree)

            print


        # We need to modify non-reaction goals to be or'd with the reactions
        if self.react_props:
            # Dedupe and make an or over all the reaction properties
            reaction_or_frag = or_([sys_(prop) for prop in self.react_props])
            # HACK: Rewrite all the goals!
            system_lines = [insert_or_before_goal(reaction_or_frag, line) for line in system_lines]
            new_generation_trees = []
            for (text, output) in generation_trees:
                new_generation_tree = [text, []]
                for command, (env_lines, sys_lines) in output:
                    new_sys_lines = \
                        [insert_or_before_goal(reaction_or_frag, line) for line in sys_lines]
                    new_generation_tree[1].append([command, [env_lines, new_sys_lines]])
                new_generation_trees.append(new_generation_tree)
            generation_trees = new_generation_trees


        # Convert sets to lists for the caller
        custom_props = list(custom_props)
        custom_sensors = list(custom_sensors)

        print "Spec generation complete."
        print "Responses:", responses
        print "Environment lines:", env_lines
        print "System lines:", system_lines
        print "Custom props:", custom_props
        print "Custom sensors:", custom_sensors
        print "Generation tree:", generation_trees
        return env_lines, system_lines, custom_props, custom_sensors, responses, generation_trees

    def _apply_metapar(self, command):
        """Generate a metapar for a command."""
        try:
            handler = self.GOALS[command.action]
        except KeyError:
            raise KeyError('Unknown action {0}'.format(command.action))

        if command.condition:
            raise ValueError("Conditionals are not currently supported :(.")
            # TODO: Re-enable conditionals
            #return self._gen_conditional(command)
        else:
            # Extract the targets from args and pass them as arguments
            return handler(command)

    def _gen_follow(self, _):
        """Generate statements for following."""
        # Env is stationary iff in the last state change our region and the env's region were stable
        env_stationary_safeties = \
            always(iff(next_(sys_(FOLLOW_STATIONARY)),
                         and_([iff(env(region), next_(env(region)))
                               for region in self.regions])))
        # Match the sensor location to ours
        follow_goals = \
            [always_eventually(implies(and_((sys_(FOLLOW_STATIONARY), env(region))), sys_(region)))
             for region in self.regions]
        return ([env_stationary_safeties] + follow_goals, [FOLLOW_SENSORS], [FOLLOW_STATIONARY], [])

    def _gen_conditional(self, action, arg_dict, condition):
        """Generate a conditional action"""
        # Validate the condition
        if condition not in self.sensors:
            raise KeyError("Unknown condition {0}".format(condition))

        condition_stmt = env(condition)
        sys_statements = [not_(condition_stmt)]
        new_props = []

        # Validate the action
        if action not in self.props and action not in self.REACTIONS:
            raise KeyError("Unknown reaction or actuator {0}".format(action))

        # Create the right type of reaction
        if action in self.props:
            # Simple actuator
            reaction_prop = action
        else:
            # Reaction proposition
            reaction_prop_name = REACT + "_" + condition
            reaction_prop = sys_(reaction_prop_name)
            self.react_props.add(reaction_prop_name)
            new_props.append(reaction_prop_name)

        # Generate the response
        if action == "go":
            # Go is unusual because the outcome is not immediately satisfiable
            destination_stmt = sys_(arg_dict[LOCATION])
            # New goal for where we should go
            go_goal = always_eventually(implies(reaction_prop, destination_stmt))
            # Safety that persists
            go_safety = \
                always(iff(next_(reaction_prop),
                             or_([reaction_prop, next_(condition_stmt)])))
            # Make sure we act immediately: []((!react & next(react) )-> stay_there)
            stay_there = always(implies(and_((not_(reaction_prop), next_(reaction_prop))),
                                          self._frag_stay()))

            sys_statements.extend([go_goal, go_safety, stay_there])
        else:
            # Otherwise we are always creating reaction safety
            sys_statements.append(always(iff(next_(condition_stmt), next_(reaction_prop))))

            # Create a reaction fragment if needed
            if action in self.REACTIONS:
                handler, targets = self.REACTIONS[action]
                args = [arg_dict[target] for target in targets]
                reaction_frag = handler(*args)
                sys_statements.append(always(implies(next_(reaction_prop), reaction_frag)))

        return (sys_statements, [], new_props, [])

    def _gen_stay(self, _):
        """Generate statements to stay exactly where you are."""
        return ([always_eventually(self._frag_stay())], [], [], [])

    def _frag_stay(self):
        """Generate fragments to reactively go somewhere."""
        return (and_([iff(sys_(region), next_(sys_(region))) for region in self.regions]))


# The return signature of the statement generators is:
# ([system lines], [env lines], [custom propositions], [custom sensors])
# TODO: These need to all be reviewed to support new command structure.

def _gen_begin(command):
    """Generate statements to begin in a location."""
    return ([sys_(command.theme.name)], [], [], [])


def _gen_patrol(command):
    """Generate statements to always eventually be in a location."""
    return ([always_eventually(sys_(command.location.name))], [], [], [])


def _gen_go(command):
    """Generate statements to go to a location once."""
    # Avoid if it's negated
    if command.negation:
        return _gen_avoid(command)
    
    region = command.location.name
    mem_prop = _prop_mem(region, VISIT)
    alo_sys = _frag_atleastonce(mem_prop, next_(sys_(region)))
    return (alo_sys, [], [mem_prop], [])


def _frag_react_go(region):
    """Generate a fragment to reactively go somewhere."""
    return (sys_(region))


def _gen_avoid(command):
    """Generate statements for avoiding a location, adding that the robot does not start there."""
    region = command.location.name
    return ([always(not_(sys_(region))), not_(sys_(region))], [], [], [])


def _gen_search(command):
    """Generate statements for searching a region."""
    region = command.location.name
    mem_prop = _prop_mem(region, SWEEP)
    cic_frag, cic_env = _frag_complete_context(SWEEP, sys_(region))
    alo_sys = _frag_atleastonce(mem_prop, cic_frag)
    return (alo_sys, cic_env, [mem_prop], [_prop_actuator_done(SWEEP)])


def _frag_atleastonce(mem_prop, fragment):
    """Generate fragments for performing an action at least once by using a memory proposition."""
    return [always_eventually(sys_(mem_prop)),
            always(iff(next_(sys_(mem_prop)), or_((sys_(mem_prop), fragment))))]


def _frag_complete_context(actuator, context_prop):
    """Generate fragments for completing an action in context."""
    actuator_done = _prop_actuator_done(actuator)
    eventually_actuator = always_eventually(env(actuator_done))
    return [and_((sys_(actuator), next_(env(actuator_done)), context_prop)), [eventually_actuator]]


def _prop_mem(region, event):
    """Generate a proposition for having visited a region."""
    return "_".join((MEM, event, region))


def _prop_actuator_done(actuator):
    """Generate a proposition for an actuator's completion."""
    return "_".join((actuator, DONE))


def format_command(command):
    """Format a command nicely for display."""
    # TODO: Should have a more general solution here.
    argument = command.location.name if command.location else \
        command.theme.name if command.theme else 'None'
    return "Action: {}, Argument: {}".format(command.action, argument)

def insert_or_before_goal(or_clause, statement):
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


if __name__ == "__main__":
    specgen = SpecGenerator()
    specgen.generate('\n'.join(sys.argv[1:]), ("bomb", "hostage", "badguy"), ("r1", "r2", "r3", "r4"),
               ("defuse", "call"))
