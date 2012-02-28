"""Generates a logical specification from natural language."""

import socket

from semantics.knowledge import Knowledge
from pipelinehost import socket_parse, DEFAULT_PORT

# LTL Constants
ALWAYS = "[]"
FINALLY = "<>"
TO = "->"
IFF = "<->"
NOT = "!"
AND = "&"
OR = "|"
ROBOT_STATE = "s."
ENV_STATE = "e."


class SpecGenerator(object):
    """Enables specification generation using natural language."""

    def __init__(self):
        # Start knowledge and connect to the NLP pipeline
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.sock.connect(('localhost', DEFAULT_PORT))
        except socket.error:
            raise IOError("Could not connect to pipelinehost on port %d. "
                          "Make sure that pipelinehost is running.")
        self.world_knowledge = Knowledge()

    def generate(self, text, sensors, regions, props):
        """Generate a logical specification from natural language and propositions."""
        # Clean unicode out of everything
        if isinstance(text, unicode):
            text = text.encode('ascii', 'ignore')
        sensors = [astr.encode('ascii', 'ignore') for astr in sensors]
        regions = [astr.encode('ascii', 'ignore') for astr in regions]
        props = [astr.encode('ascii', 'ignore') for astr in props]
        
        print "NL->LTL Generation called on:"
        print "Sensors:", sensors
        print "Props:", props
        print "Regions:", regions
        
        # Make lists for POS conversions
        force_nouns = list(regions) + list(sensors)
        force_verbs = list(props)
        
        responses = []
        for line in text.split('\n'):
            if not line:
                continue
            
            print "Sending to remote parser:", repr(line)
            parse = socket_parse(asocket=self.sock, text=line, force_nouns=force_nouns,
                                 force_verbs=force_verbs)
            print parse
            user_response, semantics_result, semantics_response, new_commands = \
                self.world_knowledge.process_parse_tree(parse, line)
            responses.append(user_response)
        
        environment_lines = []
        system_lines = ["[]<>(s.%s)" % region for region in regions]
        custom_props = []
    
        print "Spec generation complete, responses:"
        print responses
        return environment_lines, system_lines, custom_props, responses


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


def _always_finally(text):
    """Wrap text in always."""
    return _parens(ALWAYS + FINALLY + _parens(text))


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
