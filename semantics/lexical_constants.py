"""Word-related constants used by semantics."""

ENTITY_ALIASES = {'me': 'Commander',
                  'i': 'Commander'}

# Primary verbnet senses for actions
SEARCH_ACTION = "search"
GO_ACTION = "go"
GET_ACTION = "retrieve"
FOLLOW_ACTION = "follow"
SEE_ACTION = "see"
TELL_ACTION = "tell"
BEGIN_ACTION = "begin"
ACTIVATE_ACTION = "activate"
DEACTIVATE_ACTION = "deactivate"
DEFUSE_ACTION = "defuse"
AVOID_ACTION = "avoid"
PATROL_ACTION = "patrol"
CARRY_ACTION = "carry"
STAY_ACTION = "stay"

# Mapping of other verbnet senses to the same actions.
# We include the identity entries just to make things easier on the talkback side
ACTION_ALIASES = {
                  'appear': GO_ACTION,
                  'get': GET_ACTION,
                  'obtain': GET_ACTION,
                  'meander': GO_ACTION,
                  'slide': GO_ACTION,
                  'nonvehicle': GO_ACTION,
                  'escape': GO_ACTION,
                  'rummage': SEARCH_ACTION,
                  'characterize': SEE_ACTION,
                  'chase': FOLLOW_ACTION,
                  'lodge': STAY_ACTION,
                  SEARCH_ACTION: SEARCH_ACTION,
                  GO_ACTION: GO_ACTION,
                  GET_ACTION: GET_ACTION,
                  FOLLOW_ACTION: FOLLOW_ACTION,
                  SEE_ACTION: SEE_ACTION,
                  TELL_ACTION: TELL_ACTION,
                  BEGIN_ACTION: BEGIN_ACTION,
                  ACTIVATE_ACTION: ACTIVATE_ACTION,
                  DEACTIVATE_ACTION: DEACTIVATE_ACTION,
                  AVOID_ACTION: AVOID_ACTION,
                  PATROL_ACTION: PATROL_ACTION,
                  CARRY_ACTION: CARRY_ACTION,
                  STAY_ACTION: STAY_ACTION,
                  DEFUSE_ACTION: DEFUSE_ACTION,
                 }

UNDERSTOOD_SENSES = set(ACTION_ALIASES.keys())
