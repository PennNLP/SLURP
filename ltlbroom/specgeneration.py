"""Generates a logical specification from natural language."""

import socket

from semantics.processing import process_parse_tree
from pipelinehost import socket_parse, DEFAULT_PORT

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
MEM = "m"
DONE = "done"
VISIT = "visit"

# Actuators
SWEEP = "sweep"

class SpecGenerator(object):
    """Enables specification generation using natural language."""

    def __init__(self):
        # Start knowledge and connect to the NLP pipeline
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect(('localhost', DEFAULT_PORT))
        except socket.error:
            raise IOError("Could not connect to pipelinehost on port %d. "
                          "Make sure that pipelinehost is running." % DEFAULT_PORT)
        
        # Sets of propositions, accessible to all methods
        self.custom_props = set()

    def generate(self, text, sensors, regions, props):
        """Generate a logical specification from natural language and propositions."""
        # Clean unicode out of everything
        text = text.encode('ascii', 'ignore')
        sensors = [astr.encode('ascii', 'ignore') for astr in sensors]
        regions = [astr.encode('ascii', 'ignore') for astr in regions]
        props = [astr.encode('ascii', 'ignore') for astr in props]
        
        print "NL->LTL Generation called on:"
        print "Sensors:", sensors
        print "Props:", props
        print "Regions:", regions
        print "Text:", repr(text)
        print
        
        # Make lists for POS conversions, including the metapar keywords
        force_nouns = list(regions) + list(sensors)
        force_verbs = list(props) + METAPARS.keys()
        
        responses = []
        system_lines = []
        env_lines = []
        env_lines_set = set() # Used to track duplicates
        custom_props = set()
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
            
            # Build the metapars
            failure = False
            for command in new_commands:
                try:
                    new_sys_lines, new_env_lines, new_custom_props = _apply_metapar(command)
                except KeyError:
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
                # Add the statements as the children of the generation tree
                generation_tree[1].append([str(command), [new_env_lines, new_sys_lines]])
                
            # Add a true response if there were commands and no failures
            responses.append(new_commands and not failure)
                
            # Add this line's stuff to the generation tree
            generation_trees.append(generation_tree)
            
            print

        # Convert custom props into a list
        custom_props = list(custom_props)

        print "Spec generation complete."
        print "Responses:", responses
        print "Environment lines:", env_lines
        print "System lines:", system_lines
        print "Custom props:", custom_props
        print "Generation tree:", generation_trees
        return env_lines, system_lines, custom_props, responses, generation_trees


def _apply_metapar(command):
    """Generate a metapar for a command."""
    # ('go', {'Location': 'porch'})
    name, args = command
    handler, targets = METAPARS[name]
    # Extract the targets from args
    args = [args[target] for target in targets]
    return handler(*args)


def _gen_begin(region):
    """Generate statements to begin in a location."""
    return ([_sys(region)], [], [])


def _gen_patrol(region):
    """Generate statements to always eventually be in a location."""
    return ([_always_eventually(_sys(region))], [], [])


def _gen_go(region):
    """Generate statements to go to a location once."""
    mem_prop = _prop_mem(region, VISIT)
    alo_sys = _gen_atleastonce(mem_prop, _next(_sys(region)))
    return (alo_sys, [], [mem_prop])


def _gen_avoid(region):
    """Generate statements for avoiding a location, adding that the robot does not start there."""
    return ([_always(_not(_sys(region))), _not(_sys(region))], [], [])


def _gen_search(region):
    """Generate statements for searching a region."""
    mem_prop = _prop_mem(region, SWEEP)
    cic_frag, cic_env = _frag_complete_context(SWEEP, _sys(region))
    alo_sys = _gen_atleastonce(mem_prop, cic_frag)
    return (alo_sys, cic_env, [mem_prop])
    

def _gen_atleastonce(mem_prop, fragment):
    """Generate statements for perfoming an action at least once by using a memory proposition."""
    return [_always_eventually(_sys(mem_prop)), 
            _always(_iff(_next(_sys(mem_prop)), _or((_sys(mem_prop), fragment))))]


def _frag_complete_context(actuator, context_prop):
    """Generate a fragment for completing an action in context."""
    actuator_done = _prop_actuator_done(actuator)
    eventually_actuator = _always_eventually(_env(actuator_done))
    return [_and((_sys(actuator), _next(_env(actuator_done)), context_prop)), [eventually_actuator]]


def _prop_mem(region, event):
    """Generate a proposition for having visited a region."""
    return "_".join((MEM, event, region))


def _prop_actuator_done(actuator):
    """Generate a proposition for an actuator's completion."""
    return "_".join((actuator, DONE))


# MetaPARS have to be defined after all handlers have been defined
METAPARS = {'patrol': (_gen_patrol, (LOCATION,)), 'go': (_gen_go, (LOCATION,)), 
            'avoid': (_gen_avoid, (LOCATION,)), 'search': (_gen_search, (LOCATION,)),
            'begin': (_gen_begin, (THEME,))}


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
    s.generate("Go to r1.", (), ("r1",), ())
