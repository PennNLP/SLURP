"""
Handle dialog with the natural language subsystem.
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

from ltlbroom.specgeneration import speclines_from_gentree, goal_to_speclines


class DialogManager(object):
    """Supports stateful dialog with the NL system."""

    def __init__(self, generation_tree=None):
        # Store the logic generation tree if it is provided.
        self.gen_tree = generation_tree
        self.spec_lists = speclines_from_gentree(self.gen_tree) if self.gen_tree else None
        self.user_history = []
        self.system_history = []

    def tell(self, text, current_goal=None):
        """Tell the system something and return its response."""
        self.user_history.append(text)
        # Return the English of the current goal
        if self.gen_tree and current_goal:
            goal_spec_lines = goal_to_speclines(current_goal, self.spec_lists)
            explanation = " and ".join(repr(spec_lines.explanation)
                                       for spec_lines in goal_spec_lines)
            response = "I'm currently trying to {}".format(explanation)
        else:
            response = "Thank you for sharing."

        self.system_history.append(response)
        return response
