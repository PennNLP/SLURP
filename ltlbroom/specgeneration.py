"""Generates a logical specification from natural language."""

import socket

from semantics.knowledge import Knowledge
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
VISIT = "visit"

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
        self.world_knowledge = Knowledge()
        
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
        
        # Make lists for POS conversions, including the metapar keywords
        force_nouns = list(regions) + list(sensors)
        force_verbs = list(props) + METAPARS.keys()
        
        responses = []
        system_lines = []
        environment_lines = []
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
                self.world_knowledge.process_parse_tree(parse, line)
            
            # Build the metapars
            failure = False
            for command in new_commands:
                try:
                    new_sys_lines, new_env_lines, new_custom_props = _apply_metapar(command)
                except KeyError:
                    failure = True
                    continue
                
                system_lines.extend(new_sys_lines)
                new_env_lines.extend(new_env_lines)
                custom_props.update(new_custom_props)
                # Add the statements as the children of the generation tree
                generation_tree[1].append([str(command), [new_env_lines, new_sys_lines]])
                
            # Add a true response if there were commands and no failures
            responses.append(new_commands and not failure)
                
            # Add this line's stuff to the generation tree
            generation_trees.append(generation_tree)

        # Convert custom props into a list
        custom_props = list(custom_props)

        print "Spec generation complete."
        print "Responses:", responses
        print "Environment lines:", environment_lines
        print "System lines:", system_lines
        print "Custom props:", custom_props
        print "Generation tree:", generation_trees
        return environment_lines, system_lines, custom_props, responses, generation_trees


def _apply_metapar(command):
    """Generate a metapar for a command."""
    # ('go', {'Location': 'porch'})
    name, args = command
    handler, targets = METAPARS[name]
    # Extract the targets from args
    args = [args[target] for target in targets]
    return handler(*args)


def _gen_begin(region):
    """Generate a statement to begin in a location."""
    return ([_sys(region)], [], [])


def _gen_patrol(region):
    """Generate a statement to always eventually be in a location."""
    return ([_always_eventually(_sys(region))], [], [])


def _gen_go(region):
    """Generate a statement to go to a location once."""
    return ([_always_eventually(_sys(_prop_mem_visit(region))), 
             _always(_iff(_next(_sys(_prop_mem_visit(region))), _or((_next(_sys(region)),
                                                                  _sys(_prop_mem_visit(region))))))],
            [], [_prop_mem_visit(region)])


def _gen_avoid(region):
    """Generate a statement to avoid a location and that the robot does not start there."""
    return ([_always(_not(_sys(region))), _not(_sys(region))], [], [])


def _gen_search(region):
    """Generate a statement to go to a search a region."""
    return ([_always_eventually(_sys(_prop_mem_visit(region))), 
             _always(_iff(_next(_sys(_prop_mem_visit(region))), _or((_next(_sys(region)),
                                                                  _sys(_prop_mem_visit(region))))))],
            [], [_prop_mem_visit(region)])
    

def _prop_mem_visit(region):
    """Generate a proposition for having visited a region."""
    return "_".join((MEM, VISIT, region))


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
