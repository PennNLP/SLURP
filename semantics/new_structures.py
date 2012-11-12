class NewEntity:
    TYPES = ['Object', 'Location']
    TYPE_ID = -1 # Subclasses should override TYPE_ID
    def __init__(self, name = None, info = None):
        self.name = name
        self.quantifier = NewQuantifier()
        self.info = info
    def __str__(self):
        return '%s:\n'%str(self.TYPES[self.TYPE_ID]) + \
            '\t\tName:%s\n'%str(self.name) + \
            '\t\tQuantifier:\n%s\n'%str(self.quantifier) + \
            '\t\tInfo:%s'%str(self.info)
    def __repr__(self):
        return str(self)
class Object (NewEntity):
    TYPE_ID = 0
class Location (NewEntity):
    TYPE_ID = 1

class NewQuantifier:
    """A generalized quantifier class used to pick out the correct number of                                                                                                     
    entities from a universe."""
    def __init__(self, plural = None, definite = None, exhaustive = None, \
                 proportionality = 'exact', number = None):
        self.plural = plural
        self.definite = definite
        self.exhaustive = exhaustive
        self.proportionality = proportionality
        self.number = number
        self.fulfilled = True
    def __str__(self):
        return '\t\t\tPlural:%s\n'%str(self.plural) +\
               '\t\t\tDefinite:%s\n'%str(self.definite) +\
               '\t\t\tExhaustive:%s\n'%str(self.exhaustive) +\
               '\t\t\tProportionality:%s\n'%str(self.proportionality) +\
               '\t\t\tNumber:%s'%str(self.number)

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
    def __init__(self, agent, theme, patient, action, condition=None, negation = False):
        self.agent = agent
        self.theme = theme
        self.patient = patient
        self.action = action
        self.condition = condition
        self.negation = negation

    def __str__(self):
        return '\nCommand: \n' + \
               '\tAgent:\t' + str(self.agent) + '\n' + \
               '\tTheme:\t' + str(self.theme) + '\n' + \
               '\tPatient:\t' + str(self.patient) + '\n' + \
               '\tAction:' + str(self.action) + '\n' + \
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
