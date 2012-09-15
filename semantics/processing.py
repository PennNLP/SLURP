
import parsing
from lexical_constants import *
from structures import Entity, Assertion, Event, Command, WhQuery

def process_parse_tree(parse_tree_input, text_input):
    """Produces semantic interpretations of parse trees."""

    print "Processing:", repr(text_input)
    semantics_result = parsing.get_semantics_from_parse_tree(parse_tree_input)
    print "Semantics result:", semantics_result
    semantic_structures = parsing.create_semantic_structures(semantics_result)
    semantics_response = parse_semantic_structures(semantic_structures)
    print "Answer from semantics:", semantics_response
    try:
        new_commands = semantics_response[0]
        user_response = semantics_response[3]
        # Get the diff between the new and old command queue
        print "New commands:", new_commands
    except (TypeError, IndexError):
        user_response = ""
        new_commands = []
    return (user_response, semantics_result, semantics_response, new_commands)


def parse_semantic_structures(semantic_structure_list):
    """Update belief maps based on Commander's utterance, and (TODO) answer
    queries."""
    new_commands = []
    junior_known_entities = {}
    commander_known_entities = {}
    watch_list = []
    discourse_stack = []
    conditional = (['if'] in semantic_structure_list or ['when'] in semantic_structure_list)
    conditional_command = None
    response = ""
    
    for semantic_structure in semantic_structure_list:
        print "Semantic structure:", semantic_structure
        if str(semantic_structure) == "['if']" or str(semantic_structure) == "['when']":
            continue
        elif str(semantic_structure) == "or":
            continue
        else:
            discourse_stack.append(semantic_structure)

        # Add events to the watch list
        if isinstance(semantic_structure, Event):
            print "Adding the following to the semantics watch list:"
            print semantic_structure
            watch_list.append(semantic_structure)
            # Pad response if needed before adding to it.
            if response:
                response += " "
            response += "I'll let you know if I see a %s." % \
                (str(semantic_structure.entity_class.predicates['Theme'][0].value))

        # Update knowledge bases with assertions
        if isinstance(semantic_structure, Assertion):
            if semantic_structure.existential is True:
                instances = semantic_structure.entity_class.instantiate()
                for instance in instances:
                    new_entity = Entity(dict(instance.predicates.items() + \
                                    semantic_structure.predicates.items()))
                    junior_known_entities[new_entity.id] = new_entity
                    commander_known_entities[new_entity.id] = new_entity
                    
                    response += "Got it, I now know about the %s." % \
                        str(new_entity.predicates['Theme'][0].value)
                return (new_commands, watch_list, commander_known_entities, response)
            else:
                # TODO: Re-enable and make it actually work
                pass
                #instances = generate_reference_list(\
                #                semantic_structure.entity_class,
                #                commander_known_entities.values())

        # Answer YN questions
        # TODO: Re-enable
        #if isinstance(semantic_structure, YNQuery):
        #    entity_matches = self.generate_reference_list(\
        #                        semantic_structure.entity_class,\
        #                        self.commander_known_entities.values())

        #    # Instant pragmatics!
        #    return entity_matches

        # If it's a conditional, we'll add it to the last event
        # If it's not, put it on the queue to be executed when Commander says so
        if isinstance(semantic_structure, Command):
            if conditional is True:
                conditional_command = semantic_structure
                conditional = False
            else:
                _add_command_to_queue(new_commands, semantic_structure)

        # Answer WH-questions
        if isinstance(semantic_structure, WhQuery):
            # TODO: Re-enable
            pass
    
    # Add pointers to events for conditional commands
    if conditional_command is not None:
        for event in watch_list:
            if event.command is None:
                event.command = conditional_command
                                           
    return (new_commands, watch_list, commander_known_entities, response)


def _add_command_to_queue(queue, command):
    """Converts Commands to a <(action,[arguments])> structure to add to the queue, and 
    performs aliasing for entity and action names.
    Hack: Last element in predicate list is usually most accurate."""
    action = get_aliased_action(command.action)
    arguments = {}
    
    # Add all predicates to arguments and add the command
    for predicate_type, predicate_list in command.entity_class.predicates.items():
        if predicate_type != 'Agent' and predicate_list:
            arguments[predicate_type] = get_aliased_entity(str(predicate_list[-1].value))
                  
    queue.append((action, arguments))


def get_aliased_action(action_name):
    """Checks the action alias dict for a replacement name."""
    if action_name in ACTION_ALIASES:
        return ACTION_ALIASES[action_name]
    else:
        return action_name


def get_aliased_entity(entity_name):
    """Checks the entity dict for a replacement name."""
    if entity_name.lower() in ENTITY_ALIASES:
        return ENTITY_ALIASES[entity_name.lower()]
    elif entity_name == 'here':
        # TODO: Re-enable handling of 'here'
        return 'nowhere'
    else:
        return entity_name
