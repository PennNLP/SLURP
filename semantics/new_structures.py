from util import text2int

class NewEntity:
    TYPES = ['Object', 'Location']
    TYPE_ID = -1 # Subclasses should override TYPE_ID
    def __init__(self, name = None, description = None):
        self.name = name
        self.quantifier = NewQuantifier()
        # Using mutable object as default argument causes
        # it to be aliased across instances o.O
        self.description = description if description is not None else []
    def merge(self, other):
        # Merge this entity with another entity
        if other.name is not None:
            self.name = other.name
        self.quantifier.merge(other.quantifier)
        self.description.extend(other.description)
    def __str__(self):
        return '%s:\n'%str(self.TYPES[self.TYPE_ID]) + \
            '\t\tName:%s\n'%str(self.name) + \
            '\t\tQuantifier:\n%s\n'%str(self.quantifier) + \
            '\t\tDescription:%s'%str(self.description)
    def __repr__(self):
        return str(self)
class Object (NewEntity):
    TYPE_ID = 0
class Location (NewEntity):
    TYPE_ID = 1

class NewQuantifier:
    # Exactly one parameter should be specified at a time
    def __init__(self, dt = None, cd = None):
        if dt in ('any','some'):
            self.definite = False
            self.type = 'any'
            self.number = None
        elif dt in ('a', 'an'):
            self.definite = False
            self.type = 'exact'
            self.number = 1
        elif dt in ('none','no'):
            self.definite = True
            self.type = 'none'
            self.number = 0
        elif dt == 'all':
            self.definite = True
            self.type = 'all'
            self.number = None
        else: # When dt is 'the', None, or unknown
            # Lowest priority when merging
            self.definite = True
            self.type = 'exact'
            self.number = 1
        if cd != None:
            self.number = cd if cd.isdigit() else text2int(cd)
    def __str__(self):
        return '\t\t\tDefinite:%s\n'%str(self.definite) +\
               '\t\t\tType:%s\n'%str(self.type) +\
               '\t\t\tNumber:%s'%str(self.number)
    def fill_determiner(self, dt):
        self.merge(NewQuantifier(dt = dt))
    def fill_cardinal(self, cd):
        self.merge(NewQuantifier(cd = cd))
    def merge(self, other):
        # Merge quantifier with other quantifer

        # Assume combination of definite and indefinite is definite
        # e.g. some of the rooms
        self.definite = self.definite and other.definite

        # Non-exact types and numbers should take precedence
        if other.type in ('any','none','all'):
            self.type = other.type

        if other.number != 1:
            self.number = other.number
        
    def __repr__(self):
        return str(self)

class NewAssertion:
    """Asserts the existence or property of an Entity in the world."""
    def __init__(self, entity_class, predicates, existential = False):
        self.entity_class = entity_class
        self.predicates = predicates
        self.existential = existential

    def __init__(self, theme, location, existential = False):
        self.theme = theme
        self.location = location
        self.existential = existential

    def __str__(self):
        return 'Assertion: \n' + \
               '\tTheme: %s\n'%str(self.theme) + \
               '\tLocation: %s\n'%str(self.location) + \
               '\tExistential: %s\n'%str(self.existential)

    def __repr__(self):
        return str(self)
        

class NewYNQuery:
    """Yes/No queries."""
    def __init__(self, theme, location):
        self.theme = theme
        self.location = location

    def __str__(self):
        return 'YNQuery: \n' + \
               '\tTheme: %s\n'%str(self.theme) + \
               '\tLocation: %s'%str(self.location)
    def __repr__(self):
        return str(self)

class NewWhQuery:
    """A Wh-query, which contains the EntityClass in question and the predicate
    being queried."""
    def __init__(self, entity, type):
        self.entity = entity
        self.type = type

    def __str__(self):
        return 'WhQuery: \n' + \
               '\tEntity:%s\n'%str(self.entity) + \
               '\tType:%s'%str(self.type)

    def __repr__(self):
        return str(self)

class NewCommand:
    """A Command for Junior to do something."""
    def __init__(self, agent, theme, patient, location, action, condition=None, negation = False):
        self.agent = agent
        self.theme = theme
        self.patient = patient
        self.location = location
        self.action = action
        self.condition = condition
        self.negation = negation

    def __str__(self):
        return '\nCommand: \n' + \
               '\tAgent:\t' + str(self.agent) + '\n' + \
               '\tAction:' + str(self.action) + '\n' + \
               '\tTheme:\t' + str(self.theme) + '\n' + \
               '\tPatient:\t' + str(self.patient) + '\n' + \
               '\tLocation:\t' + str(self.location) + '\n' + \
               '\tCondition:' + str(self.condition) + '\n' + \
               '\tNegation:' + str(self.negation)
    def __repr__(self):
        return str(self)

class NewEvent:
    """An event in the environment that can trigger a command."""
    def __init__(self, entity_class, sensor):
        self.entity_class = entity_class
        self.sensor = sensor
        self.command = None

    def __str__(self):
        return 'Event:\n' + \
               '\tEntityClass: ' + str(self.entity_class) + '\n' +\
               '\tSensor: ' + str(self.sensor) +'\n' +\
               '\tCommand: ' + str(self.command)

    def __repr__(self):
        return str(self)
