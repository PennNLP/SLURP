"""
Classes used for storing the semantic
structures generated by the semantic parser.
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

from semantics.util import text2int


class Entity(object):
    """Parent class of entities"""

    TYPES = ['Object', 'Location']
    TYPE_ID = -1  # Subclasses should override TYPE_ID

    def __init__(self, name=None, description=None):
        self.name = name
        self.quantifier = Quantifier()
        # Using mutable object as default argument causes
        # it to be aliased across instances
        self.description = description if description is not None else []

    def merge(self, other):
        """Merge this entity with another entity"""
        if other.name is not None:
            self.name = other.name
        self.quantifier.merge(other.quantifier)
        self.description.extend(other.description)

    def readable(self):
        return '%s %s' % (self.quantifier.readable(), self.name)

    def __str__(self, lvl=0):
        if self.name == '*':
            return self.name
        indent = '\t'*(lvl)
        return str(self.TYPES[self.TYPE_ID]) + '\n'+ \
            indent + '\tName: ' + self.name + '\n' + \
            indent + '\tQuantifier: ' + (self.quantifier.__str__(lvl + 1) if self.quantifier else '') + '\n' + \
            indent + '\tDescription: ' + str(self.description)

    def __repr__(self):
        return str(self)


class ObjectEntity(Entity):
    """Class representing an object entity"""

    TYPE_ID = 0


class Location(Entity):
    """Class representing a location entity"""

    TYPE_ID = 1


class Quantifier(object):
    """Class representing a quantifier"""

    # Exactly one parameter should be specified at a time
    def __init__(self, dt=None, cd=None):
        if dt in ('any', 'some'):
            self.definite = False
            self.type = 'any'
            self.number = None
        elif dt in ('a', 'an'):
            self.definite = False
            self.type = 'exact'
            self.number = 1
        elif dt in ('none', 'no'):
            self.definite = True
            self.type = 'none'
            self.number = 0
        elif dt == 'all':
            self.definite = True
            self.type = 'all'
            self.number = None
        else:  # When dt is 'the', None, or unknown
            # Lowest priority when merging
            self.definite = True
            self.type = 'exact'
            self.number = 1
        if cd != None:
            self.number = cd if cd.isdigit() else text2int(cd)

    def readable(self):
        if self.definite:
            if self.type == 'all':
                return self.type
            if self.number == 0:
                return 'no'
            if self.number == 1:
                return 'the'
            else:
                return str(self.number)
        else:
            if self.number == 1:
                return 'a'
            else:
                return 'any'

    def __str__(self, lvl=0):
        indent = '\t'*(lvl)
        return '\n' + indent + '\tDefinite:%s\n' % str(self.definite) + \
               indent + '\tType:%s\n' % str(self.type) +\
               indent + '\tNumber:%s' % str(self.number)

    def fill_determiner(self, dt):
        """Fills self with a determiner by merging it with
        a new quantifier created with that determiner"""
        self.merge(Quantifier(dt=dt))

    def fill_cardinal(self, cd):
        """Fills self with a cardinal number  by merging it with
        a new quantifier created with that cardinal number"""
        self.merge(Quantifier(cd=cd))

    def merge(self, other):
        """Merge quantifier with other quantifer"""

        # Assume combination of definite and indefinite is definite
        # e.g. some of the rooms
        self.definite = self.definite and other.definite

        # Non-exact types and numbers should take precedence
        if other.type in ('any', 'none', 'all'):
            self.type = other.type

        if other.number != 1:
            self.number = other.number

    def __repr__(self):
        return str(self)


class Assertion(object):
    """Asserts the existence or property of an Entity in the world."""

    def __init__(self, theme, location, existential=False):
        self.theme = theme
        self.location = location
        self.existential = existential

    def __str__(self, lvl=0):
        indent = '\t'*(lvl)
        return '\n' + indent + '\tTheme: %s\n' % str(self.theme, lvl + 1) + \
               indent + '\tLocation: %s\n' % str(self.location, lvl + 1) + \
               indent + '\tExistential: %s\n' % str(self.existential)

    def __repr__(self):
        return str(self)


class Query(object):
    """Base class for all queries"""

    def __init__(self, theme):
        self.theme = theme

    def check_fact(self, fact):
        """ Returns response if query is answered by fact. Returns none otherwise"""
        return None

    def __repr__(self):
        return str(self)

    def __str__(self, lvl=0):
        indent = '\t'*(lvl)
        return '\n' + indent + 'LocationQuery: \n' + \
               indent + '\tTheme:%s\n' % str(self.theme, lvl + 1)


class YNQuery(Query):
    """Yes/No queries."""
    def __init__(self, theme, location):
        self.theme = theme
        self.location = location

    def __str__(self, lvl=0):
        indent = '\t'*(lvl)
        return 'YNQuery: \n' + \
               indent + '\tTheme: %s\n' % str(self.theme, lvl + 1) + \
               indent + '\tLocation: %s' % str(self.location, lvl + 1)


class LocationQuery(Query):
    """Where queries"""
    pass


class StatusQuery(Query):
    """Status queries"""
    pass


class EntityQuery(Query):
    """Who/What queries"""
    def __init__(self, location):
        self.location = location

    def __str__(self, lvl=0):
        indent = '\t'*(lvl)
        return 'EntityQuery: \n' + \
               indent + '\tLocation:%s\n' % str(self.location, lvl + 1)


class Command(object):
    """A Command for Junior to do something."""

    def __init__(self, agent, theme, patient, location, action,
                 condition=None, negation=False):
        self.agent = agent
        self.theme = theme
        self.patient = patient
        self.location = location
        self.action = action
        self.condition = condition
        self.negation = negation

    def __str__(self, lvl=0):
        indent = '\t'*(lvl)
        
        return '\nCommand: \n' + \
               indent + '\tAgent: ' + (self.agent.__str__(lvl + 1) if self.agent else '') + '\n' + \
               indent + '\tAction: ' + str(self.action)  + '\n' + \
               indent + '\tTheme:' + (self.theme.__str__(lvl + 1) if self.theme else '') + '\n' + \
               indent + '\tPatient:' + (self.patient.__str__(lvl + 1) if self.patient else '') + '\n' + \
               indent + '\tLocation: ' + (self.location.__str__(lvl + 1) if self.location else '') + '\n' + \
               indent + '\tCondition: ' + (self.condition.__str__(lvl + 1) if self.condition else '') + '\n' + \
               indent + '\tNegation: ' + str(self.negation)

    def __repr__(self):
        return str(self)


class Event(object):
    """An event in the environment."""

    def __init__(self, entity, sensor):
        self.entity = entity
        self.sensor = sensor

    def __str__(self, lvl=0):
        indent = '\t'*(lvl)
        return 'Event:\n' + \
               indent + '\tSensor: ' + str(self.sensor) + '\n' + \
               indent + '\tEntity: ' + (self.entity.__str__(lvl + 1) if self.entity else '')

    def __repr__(self):
        return str(self)
