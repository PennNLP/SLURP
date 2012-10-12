"""
Generates a logical specification from natural language.

"""

# Copyright (C) 2010-2012 Constantine Lignos
#
# This file is a part of SLURP.
#
# SLURP is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# SLURP is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with SLURP.  If not, see <http://www.gnu.org/licenses/>.

import socket
import sys

from semantics.processing import process_parse_tree, CONDITION_ARGUMENT
from pipelinehost import socket_parse, DEFAULT_PORT

# Debug constants
SEMANTICS_DEBUG = False

# LTL Constants
ALWAYS = "[]"
EVENTUALLY = "<>"
TO = "->"
IFF = "<->"
NOT = "!"
AND = "&"
OR = "|"
ROBOT_STATE = "s."
ENV_STATE = "e."

# Semantics constants
THEME = "Theme"
LOCATION = "Location"
PATIENT = "Patient"
SOURCE = "Source"
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

    def __init__(self, hostname='localhost'):
        # Start knowledge and connect to the NLP pipeline
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect((hostname, DEFAULT_PORT))
        except socket.error:
            raise IOError("Could not connect to pipelinehost on %s:%d. "
                          "Make sure that pipelinehost is running." % 
                            (hostname, DEFAULT_PORT))
            
        # Handlers
        self.GOALS = {'patrol': (_gen_patrol, (LOCATION,)), 'go': (_gen_go, (LOCATION,)), 
                     'avoid': (_gen_avoid, (LOCATION,)), 'search': (_gen_search, (LOCATION,)),
                     'begin': (_gen_begin, (THEME,)), 'follow': (self._gen_follow, ()),
                     'stay': (self._gen_stay, ())}
        
        self.REACTIONS = {'go': (_frag_react_go, (LOCATION,)), 'stay': (self._frag_stay, ())}
        
        # Information about the scenario will be updated at generation time
        self.sensors = None
        self.regions = None
        self.props = None
        self.react_props = set()

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
            
            # Lowercase and strip the text before using it
            line = line.strip().lower()
            
            print "Sending to remote parser:", repr(line)
            parse = socket_parse(asocket=self.sock, text=line, force_nouns=force_nouns,
                                 force_verbs=force_verbs)
            print parse
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
            reaction_or_frag = _or([_sys(prop) for prop in self.react_props])
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
        # ('go', {'Location': 'porch'})
        name, arg_dict = command
        try:
            handler, targets = self.GOALS[name]
        except KeyError:
            raise KeyError('Unknown action {0}'.format(name))

        
        if CONDITION_ARGUMENT in arg_dict:
            return self._gen_conditional(name, arg_dict, arg_dict[CONDITION_ARGUMENT])
        else:
            # Extract the targets from args and pass them as arguments
            args = [arg_dict[target] for target in targets]
            return handler(*args)
    
    def _gen_follow(self):
        """Generate statements for following."""
        # Env is stationary iff in the last state change our region and the env's region were stable
        env_stationary_safeties = \
            _always(_iff(_next(_sys(FOLLOW_STATIONARY)), 
                         _and([_iff(_env(region), _next(_env(region)))
                               for region in self.regions])))
        # Match the sensor location to ours
        follow_goals = \
            [_always_eventually(_implies(_and((_sys(FOLLOW_STATIONARY), _env(region))), _sys(region)))
             for region in self.regions]
        return ([env_stationary_safeties] + follow_goals, [FOLLOW_SENSORS], [FOLLOW_STATIONARY], [])

    def _gen_conditional(self, action, arg_dict, condition):
        """Generate a conditional action"""
        # Validate the condition
        if condition not in self.sensors:
            raise KeyError("Unknown condition {0}".format(condition))
        
        condition_stmt = _env(condition)
        sys_statements = [_not(condition_stmt)]
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
            reaction_prop = _sys(reaction_prop_name)
            self.react_props.add(reaction_prop_name)
            new_props.append(reaction_prop_name)
            
        # Generate the response
        if action == "go":
            # Go is unusual because the outcome is not immediately satisfiable
            destination_stmt = _sys(arg_dict[LOCATION])
            # New goal for where we should go
            go_goal = _always_eventually(_implies(reaction_prop, destination_stmt))
            # Safety that persists
            # TODO: Consider having this self-deactivate
            go_safety = \
                _always(_iff(_next(reaction_prop), 
                             _or([reaction_prop, _next(condition_stmt)])))
            sys_statements.extend([go_goal, go_safety])
        else:
            # Otherwise we are always creating reaction safety
            sys_statements.append(_always(_iff(_next(condition_stmt), _next(reaction_prop))))
            
            # Create a reaction fragment if needed
            if action in self.REACTIONS:
                handler, targets = self.REACTIONS[action]
                args = [arg_dict[target] for target in targets]
                reaction_frag = handler(*args)
                sys_statements.append(_always(_implies(_next(reaction_prop), reaction_frag)))

        return (sys_statements, [], new_props, [])

    def _gen_stay(self):
        """Generate statements to stay exactly where you are."""
        return ([_always_eventually(self._frag_stay())], [], [], [])
    
    def _frag_stay(self):
        """Generate fragments to reactively go somewhere."""
        return (_and([_iff(_sys(region), _next(_sys(region))) for region in self.regions]))


# The return signature of the statement generators is:
# ([system lines], [env lines], [custom propositions], [custom sensors])

def _gen_begin(region):
    """Generate statements to begin in a location."""
    return ([_sys(region)], [], [], [])


def _gen_patrol(region):
    """Generate statements to always eventually be in a location."""
    return ([_always_eventually(_sys(region))], [], [], [])


def _gen_go(region):
    """Generate statements to go to a location once."""
    mem_prop = _prop_mem(region, VISIT)
    alo_sys = _frag_atleastonce(mem_prop, _next(_sys(region)))
    return (alo_sys, [], [mem_prop], [])


def _frag_react_go(region):
    """Generate a fragment to reactively go somewhere."""
    return (_sys(region))


def _gen_avoid(region):
    """Generate statements for avoiding a location, adding that the robot does not start there."""
    return ([_always(_not(_sys(region))), _not(_sys(region))], [], [], [])


def _gen_search(region):
    """Generate statements for searching a region."""
    mem_prop = _prop_mem(region, SWEEP)
    cic_frag, cic_env = _frag_complete_context(SWEEP, _sys(region))
    alo_sys = _frag_atleastonce(mem_prop, cic_frag)
    return (alo_sys, cic_env, [mem_prop], [_prop_actuator_done(SWEEP)])


def _frag_atleastonce(mem_prop, fragment):
    """Generate fragments for performing an action at least once by using a memory proposition."""
    return [_always_eventually(_sys(mem_prop)), 
            _always(_iff(_next(_sys(mem_prop)), _or((_sys(mem_prop), fragment))))]


def _frag_complete_context(actuator, context_prop):
    """Generate fragments for completing an action in context."""
    actuator_done = _prop_actuator_done(actuator)
    eventually_actuator = _always_eventually(_env(actuator_done))
    return [_and((_sys(actuator), _next(_env(actuator_done)), context_prop)), [eventually_actuator]]


def _prop_mem(region, event):
    """Generate a proposition for having visited a region."""
    return "_".join((MEM, event, region))


def _prop_actuator_done(actuator):
    """Generate a proposition for an actuator's completion."""
    return "_".join((actuator, DONE))


def format_command(command):
    """Format a command nicely for display."""
    # Structure: ('follow', {'Theme': 'Commander'})
    action, arg_dict = command
    action = action.lower()
    return (", ".join(["Action: {0}".format(action)] + 
                      ["{0}: {1}".format(key, val) for key, val in sorted(arg_dict.items())]))


def insert_or_before_goal(or_clause, statement):
    """Inserts a statement to be OR'd against what is already in the goal."""
    # Your hack is bad and you should feel bad.
    if ALWAYS + EVENTUALLY in statement and REACT not in statement:
        return statement.replace(ALWAYS + EVENTUALLY + '(', 
                                 ALWAYS + EVENTUALLY + '(' + or_clause + _space(OR))
    else:
        return statement


def _space(text):
    """Wrap text in spaces."""
    return " " + text + " "


def _and(propostions, delim=''):
    """Add logical and to the arguments"""
    return _parens((_space(AND) + delim).join(propostions))


def _or(propostions, delim=''):
    """Add logical or to the arguments"""
    return _parens((_space(OR) + delim).join(propostions))


def _parens(text):
    """Wrap text in parens."""
    return "(" + text + ")"


def _always(text):
    """Wrap text in always."""
    return _parens(ALWAYS + _parens(text))


def _always_eventually(text):
    """Wrap text in always."""
    return _parens(ALWAYS + EVENTUALLY + _parens(text))


def _not(text):
    """Add a not operator in front of a string."""
    return NOT + text


def _sys(text):
    """Add a robot state marker in front of a string."""
    return ROBOT_STATE + text


def _env(text):
    """Add an environment state marker in front of a string."""
    return ENV_STATE + text


def _next(text):
    """Wrap text in next()."""
    return "next" + _parens(text)


def _mutex(items):
    """Create a system proposition mutex over the given items."""
    return _always(_or([_and([_next(item1)] +
                             [_not(_next(item2)) for item2 in items if item2 != item1])
                    for item1 in items], "\n    "))


def _iff(prop1, prop2):
    """Connect two propositions with if and only if."""
    return _parens(prop1 + _space(IFF) + prop2)


def _implies(prop1, prop2):
    """Connect two propositions with implies."""
    return _parens(prop1 + _space(TO) + prop2)


if __name__ == "__main__":
    s = SpecGenerator()
    s.generate('\n'.join(sys.argv[1:]), ("bomb", "hostage", "badguy"), ("r1", "r2", "r3", "r4"),
               ("defuse", "call"))
