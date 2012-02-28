"""Generates a logical specification from natural language."""


from semantics.knowledge import Knowledge
from pennpipeline import parse_text, init_pipes, close_pipes

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


# TODO: Currently SpecGenerator assumes it's a singleton object
class SpecGenerator(object):
    """Enables specification generation using natural language."""

    instantiated = False

    def __init__(self):
        if SpecGenerator.instantiated:
            raise NotImplementedError("Can only instantiate one SpecGenerator.")
        SpecGenerator.instantiated = True

        # Start up the NLP pipeline and knowledge
        init_pipes()
        self.world_knowledge = Knowledge()

    def __del__(self):
        # We put this check as it may already be undefined during interpreter shutdown, but there's
        # no guarantee this will succeed during shutdown anyway.
        if close_pipes:
            close_pipes()

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
        
        # Make sets for POS coercions
        force_nouns = set(list(regions) + list(sensors))
        force_verbs = set(props)
        
        responses = []
        for line in text.split('\n'):
            if not line:
                continue
            
            print "Parsing:", repr(line)
            parse = parse_text(text, force_nouns, force_verbs)
            print parse
            print self.world_knowledge.process_parse_tree(parse, text)
            responses.append("Ok!")
        
        environment_lines = []
        system_lines = ["[]<>(s.%s)" % region for region in regions]
        custom_props = []
    
        print "Spec generation complete, responses:"
        print responses
        return environment_lines, system_lines, custom_props


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
