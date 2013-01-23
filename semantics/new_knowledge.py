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
    def __init__(self):
        self.facts = []
        self.last_theme = None
    def process_semantic_structures(self, semantic_structures):
        response = ''
        for structure in semantic_structures:
            if isinstance(structure, Assertion):
                new_fact = self.assimilate(structure)
                response = new_fact.readable()
            elif isinstance(structure, Query):
                response = self.query(structure)
            # Assertions, Queries, and Commands have themes that may be referenced later
            if isinstance(structure, Assertion) or isinstance(structure, Query) or isinstance(structure, Command):
                # Only replace resolved entities
                if structure.theme and not is_pronoun(structure.theme.name):
                    self.last_theme = structure.theme
        return response

    def assimilate(self, assertion):
        new_fact = LocationFact(assertion)
        self.facts.append(new_fact)
        return new_fact

    def query(self, query):
        """Reponds to a given query"""
        responses = [r for r in (
            fact.query(query) for fact in self.facts) if r is not None]
        if len(responses) > 0:
            return '\n'.join(responses)
        elif isinstance(query, LocationQuery) or isinstance(query, YNQuery):
            return "I don't know about %s" % query.theme.readable()
        elif isinstance(query, EntityQuery):
            return "I don't know about %s" % query.location.readable()

    def fill_commands(self, commands):
        for c in commands:
            if isinstance(c, Command):
                if c.theme and is_pronoun(c.theme.name) and self.last_theme:
                    c.theme.name = self.last_theme
                if c.destination and not c.source:
                    for fact in self.facts:
                        if isinstance(fact, LocationFact) and c.theme and fact.theme.name == c.theme.name:
                            c.source = fact.location
    def readable(self):
        return '\n'.join(f.readable() for f in self.facts)


class Fact:
    def __init__(self):
        pass

    def query(self, query):
        """Returns a reponse if fact answers query, none otherwise"""
        return None


class LocationFact(Fact):
    def __init__(self, assertion):
        self.theme = assertion.theme
        self.location = assertion.location

    def readable(self):
        return '{!r} is/are in {!r}'.format(self.theme.readable(), self.location.readable())

    def query(self, query):
        if isinstance(query, LocationQuery):
            if query.theme.name == self.theme.name:
                return self.readable()
        elif isinstance(query, YNQuery):
            if query.theme.name == self.theme.name:
                if query.location.name == self.location.name:
                    return 'Yes, %s' % self.readable()
                else:
                    return 'No, %s' % self.readable()
        elif isinstance(query, EntityQuery):
            if query.location.name == self.location.name:
                return self.readable()
        return None
