"""Word-related constants used by semantics."""

# Copyright (C) 2011-2013 Constantine Lignos and Ian Perera
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

ENTITY_ALIASES = {'me': 'Commander',
                  'i': 'Commander'}

# Primary verbnet senses for actions
SEARCH_ACTION = "search"
GO_ACTION = "go"
GET_ACTION = "retrieve"
FOLLOW_ACTION = "follow"
SEE_ACTION = "see"
BEGIN_ACTION = "begin"
ACTIVATE_ACTION = "activate"
DEACTIVATE_ACTION = "deactivate"
DEFUSE_ACTION = "defuse"
AVOID_ACTION = "avoid"
PATROL_ACTION = "patrol"
CARRY_ACTION = "carry"
STAY_ACTION = "stay"
BE_ACTION = "be"

# Mapping of other verbnet senses to the same actions.
# We include the identity entries just to make things easier on the talkback side
ACTION_ALIASES = {
                  'appear': GO_ACTION,
                  'get': GET_ACTION,
                  'obtain': GET_ACTION,
                  'meander': GO_ACTION,
                  'slide': GO_ACTION,
                  'nonvehicle': GO_ACTION,
                  'enter': GO_ACTION,
                  'rummage': SEARCH_ACTION,
                  'characterize': SEE_ACTION,
                  'chase': FOLLOW_ACTION,
                  'lodge': STAY_ACTION,
                  SEARCH_ACTION: SEARCH_ACTION,
                  GO_ACTION: GO_ACTION,
                  GET_ACTION: GET_ACTION,
                  FOLLOW_ACTION: FOLLOW_ACTION,
                  SEE_ACTION: SEE_ACTION,
                  BEGIN_ACTION: BEGIN_ACTION,
                  ACTIVATE_ACTION: ACTIVATE_ACTION,
                  DEACTIVATE_ACTION: DEACTIVATE_ACTION,
                  AVOID_ACTION: AVOID_ACTION,
                  PATROL_ACTION: PATROL_ACTION,
                  CARRY_ACTION: CARRY_ACTION,
                  STAY_ACTION: STAY_ACTION,
                  DEFUSE_ACTION: DEFUSE_ACTION,
                  BE_ACTION: BE_ACTION,
                 }

UNDERSTOOD_SENSES = set(ACTION_ALIASES.keys())
