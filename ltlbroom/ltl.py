"""
Constants used in LTL generation.
"""

# Copyright (C) 2011-2013 Ian Perera and Constantine Lignos
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


def space(text):
    """Wrap text in spaces."""
    return " " + text + " "


def and_(propostions, delim=''):
    """Add logical and to the arguments"""
    return parens((space(AND) + delim).join(propostions))


def or_(propostions, delim=''):
    """Add logical or to the arguments"""
    return parens((space(OR) + delim).join(propostions))


def parens(text):
    """Wrap text in parens."""
    return "(" + text + ")"


def always(text):
    """Wrap text in always."""
    return parens(ALWAYS + parens(text))


def always_eventually(text):
    """Wrap text in always."""
    return parens(ALWAYS + EVENTUALLY + parens(text))


def not_(text):
    """Add a not operator in front of a string."""
    return NOT + text


def sys_(text):
    """Add a robot state marker in front of a string."""
    return ROBOT_STATE + text


def env(text):
    """Add an environment state marker in front of a string."""
    return ENV_STATE + text


def next_(text):
    """Wrap text in next()."""
    return "next" + parens(text)


def mutex_(items, include_all_off=False):
    """Create a system proposition mutex over the given items."""
    return or_([and_([item1] +
                     [not_(item2) for item2 in items if item2 != item1])
                for item1 in items] +
               ([and_([not_(item) for item in items])] if include_all_off else []))


def iff(prop1, prop2):
    """Connect two propositions with if and only if."""
    return parens(prop1 + space(IFF) + prop2)


def implies(prop1, prop2):
    """Connect two propositions with implies."""
    return parens(prop1 + space(TO) + prop2)
