"""Contains class that build a knowledge base"""

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

from semantics.new_structures import Assertion, Query, YNQuery, \
    LocationQuery, EntityQuery, Command
from semantics.util import is_pronoun

class KnowledgeBase:
    """Stores knowledge about the world"""
    def __init__(self, other_agents=None):
        self.facts = [MapFact()]
        if other_agents:
            self.facts.extend(KnowledgeFact(agent) for agent in other_agents)
        self.last_object = None
        self.last_location = None
    def process_semantic_structures(self, semantic_structures, source=None):
        """Processes semantic structures and returns a response
        if given a query"""

        response = ''
        for structure in semantic_structures:
            if isinstance(structure, Assertion):
                self.assimilate(structure, source)
                response = 'Got it. %s' % structure.readable()
            elif isinstance(structure, Query):
                response = self.query(structure)
            # Assertions, Commands have themes and locations that 
            # may be referenced later
            if isinstance(structure, Command):
                # Only replace resolved object 
                # (theme has priority over condition)
                if structure.theme and not is_pronoun(structure.theme.name):
                    self.last_object = structure.theme
                elif structure.condition and structure.condition.theme \
                        and not is_pronoun(structure.condition.theme.name):
                    self.last_object = structure.condition.theme
                # Only replace resolved locations 
                # (theme has priority over condition)
                if structure.location and structure.location.name != 'there':
                    self.last_location = structure.location
                elif structure.condition and \
                        isinstance(structure.condition, Assertion) and \
                        structure.condition.location and \
                        structure.condition.location.name != 'there':
                    self.last_location = structure.condition.location
            elif isinstance(structure, Assertion):
                if structure.theme and not is_pronoun(structure.theme.name):
                    self.last_object = structure.theme
                if structure.location and structure.location.name != 'there':
                    self.last_location = structure.location
        return response

    def assimilate(self, assertion, source):
        """ Assimilates new piece of knowledge (Assertion) """
        for f in self.facts:
            f.assimilate(assertion, source)

    def query(self, query):
        """Reponds to a given query"""
        # Ignore queries like "Where are you?"
        if isinstance(query, LocationQuery) and query.theme.name == 'you':
            return None
        responses = [r for r in (
            fact.query(query) for fact in self.facts) if r is not None]
        if len(responses) > 0:
            return '\n'.join(responses)
        return None

    def fill_commands(self, commands):
        """Fills in underspecified fields
        based on current knowledge"""
        for c in commands:
            if isinstance(c, Command):
                if self.last_object:
                    if c.theme and is_pronoun(c.theme.name):
                        c.theme.name = self.last_object.name
                    if c.patient and is_pronoun(c.patient.name):
                        c.patient.name = self.last_object.name
                    if c.condition and c.condition.theme\
                            and is_pronoun(c.condition.theme.name):
                        c.condition.theme.name = self.last_object.name
                if self.last_location:
                    if c.location and c.location.name == 'there':
                        c.location.name = self.last_location.name
                for f in self.facts:
                    if isinstance(f, MapFact):
                        if c.destination and not c.source:
                            # Fill in source
                            if c.theme:
                                result = f.query_map(None, c.theme)
                                if len(result) > 0:
                                    c.source = result[0]
                        if not c.location and not c.destination \
                                and not c.source:
                            # Fill in location
                            if c.theme:
                                result = f.query_map(None, c.theme)
                                if len(result) > 0:
                                    c.location = result[0]
    def __str__(self):
        return '\n'.join(str(f) for f in self.facts)

class Fact:
    """Base class for facts"""
    def __init__(self):
        pass

    def query(self, query):
        """"Override this in subclasses"""
        return None

    def assimilate(self, assertion, source):
        """Override this in subclasses"""
        pass

class MapFact(Fact):
    """Spatial map of the environment"""
    def __init__(self):
        # Mapping from location to a set of entities
        self.env_map = {}

    def query_map(self, location, theme):
        """"Returns the missing argument. 
        If both arguments are present, 
        returns whether that mapping is found"""
        if not location and not theme:
            return None
        elif not theme:
            return self.env_map.get(location, None)
        elif not location:
            return [location for location, entities in \
                        self.env_map.items() if theme in entities]
        else:
            # Both arguments are present
            if location in self.env_map:
                return theme in self.env_map[location]
            else:
                return None

    def query(self, query):
        """Query the database with a semantic structure"""
        if isinstance(query, YNQuery):
            result = self.query_map(query.location, query.theme)
            if result:
                return 'Yes, {!r} is/are in {!r}.'.format(\
                    query.theme.readable(), query.location.readable())
            else:
                explicit_response = 'No, {!r} is/are not in {!r}.'.format(\
                    query.theme.readable(), query.location.readable())
                # Then find out where it is
                return explicit_response + ' ' + \
                    self.query(LocationQuery(query.theme))
        elif isinstance(query, EntityQuery):
            result = self.query_map(query.location, None)
            if result:
                return '{!r} is/are in {!r}'.format(\
                    ', '.join(entity.readable() for entity in result),\
                        query.location.readable())
            else:
                return 'I don\'t know about anything in {!r}'.format(\
                    query.location.readable())
        elif isinstance(query, LocationQuery):
            result = self.query_map(None, query.theme)
            if len(result) == 0:
                return 'I don\'t know where {!r} is/are'.format(\
                    query.theme.readable())
            else:
                return '{!r} is/are in {!r}'.format(\
                    query.theme.readable(), ', '.join(\
                        location.readable() for location in result))

    def assimilate(self, assertion, source):
        if assertion.location not in self.env_map:
            self.env_map[assertion.location] = set()
        self.env_map[assertion.location].add(assertion.theme)

    def __str__(self):
        return 'Environment Map: \n\t' + '\n'.join(
            '%s: %s' % (location.name, str([e.name for e in entities]))\
                for location, entities in self.env_map.items())

class KnowledgeFact(Fact):
    """Mental model of another agent"""
    def __init__(self, agent):
        self.agent = agent
        self.kb = KnowledgeBase()

    def assimilate(self, assertion, source):
        """Include this piece of knowledge if
        it came from the agent of interest"""
        if source == self.agent:
            self.kb.assimilate(assertion, source)

    def __str__(self):
        return 'Agent Knowledge: \n\t' + \
            '%s knows: %s' % (self.agent, str(self.kb))
