"""Stores environmental context and shared knowledge of Junior for Pragbot.

State string assumed to be:
"room=<room#>;bombs=<#bombs>;hostages=<#hostages>;badguys=<#badguys>" 

Ian Perera June 2011"""

import re
from collections import defaultdict

import parsing
from structures import Predicate, Entity, Assertion, Event, YNQuery, Command, WhQuery
from lexical_constants import *
try:
    from subtle_msgs.msg import Fiducial
except ImportError:
    print "Warning: Semantics is running without a connection to the robot environment."
    Fiducial = None

DEFAULT_LOCATION = "unknown_current_location"

        
class Knowledge:
    """Encompasses world knowledge and handles IO with text input and sensor
    data."""

    def __init__(self, nlmaster=None):
        # Mapping from id to Entity
        self.junior_known_entities = {}
        self.commander_known_entities = {}

        _junior_entity = Entity({'EntityType' : [Predicate('EntityType', 'Junior')],
                                  'Location' : [Predicate('Location', 'unknown')]})
        self.junior_id = _junior_entity.id
        self.junior_known_entities[self.junior_id] = _junior_entity

        # Stores incoming semantic structures on a stack to resolve underspecifications
        self.discourse_stack = []

        # Junior's mentioned entities
        self.junior_discourse_stack = []

        # Stores events to look for
        self.watch_list = []

        # Stores commands to be sent to LTL when Commander finishes plan
        self.command_queue = []
        
        # Stores the command queue after each utterance is processed so it can be diffed.
        self.old_command_queue = []

        # These will be filled in later by a caller
        self.direction_proxy = None
        self.map_proxy = None

        # Hold on to nlmaster
        self.nlmaster = nlmaster

    def process_parse_tree(self, parse_tree_input, text_input):
        """Produces semantic interpretations of parse trees."""

        print "Processing:", repr(text_input)
        text_input_lower = text_input.lower()
        if 'orders' in text_input_lower or 'execute' in text_input_lower or \
            'make it so' in text_input_lower or \
            ('that' in text_input_lower and 'all' in text_input_lower):
            # Execute the orders
            if self.nlmaster:
                print "Calling state manager to process orders."
                unused, response = self.nlmaster.state_mgr.process_orders(self.command_queue)
                # Clear the orders regardless of success
                self.command_queue = []
                self.old_command_queue = []
                return (response, None, None, None)
            else:
                commands = str(self.command_queue)
                self.command_queue = []
                self.old_command_queue = []
                return ("I'd execute these orders if I had a state manager: %s." %
                        commands, None, None, None)
        elif ('get to you' in text_input_lower or 'get there' in text_input_lower or
              'find you' in text_input_lower):
            # Return directions for how to get there
            prep_match = re.search(r"from the ([a-zA-Z0-9]+)", text_input_lower)
            directions = None
            if prep_match and self.direction_proxy:
                # Get the room from the re and query the server
                room = prep_match.group(1)
                # You can use the commented out line if give_directions isn't working
                directions = self.direction_proxy.give_directions(room)

            # Take care of None returns from give_directions and all other cases
            if not directions:
                directions = "Sorry, I don't know how you'd get here."
            return (directions, None, None, None)
        elif 'stop' in text_input_lower:
            if self.nlmaster:
                self.nlmaster.state_mgr.stop()
                return ("Aye sir, full stop. I'm in the %s." % self.nlmaster.state_mgr.location, 
                        None, None, None)
            else:
                return ("Aye sir, full stop.", None, None, None)
        elif 'nevermind' in text_input_lower or 'belay' in text_input_lower:
            # Clear the command queue
            self.command_queue = []
            self.old_command_queue = []
            return ("Okay, I'll belay those orders.", None, None, None)   
        else:
            # Actually do semantic parsing
            semantics_result = parsing.extract_frames_from_parse(parse_tree_input)
            semantic_structures = parsing.create_semantic_structures(semantics_result)
            semantics_response = self.parse_semantic_structures(semantic_structures)
            print "Answer from semantics:", semantics_response
            try:
                user_response = semantics_response[3]
                # Get the diff between the new and old command queue
                new_commands = self.command_queue[len(self.old_command_queue):]
                print "New commands:", new_commands
                self.old_command_queue = self.command_queue[:]
            except (TypeError, IndexError):
                user_response = ""
                new_commands = []
            return (user_response, semantics_result, semantics_response, new_commands)

    def process_sensor_data(self, sensor_input):
        """Takes in sensor data and updates the world knowledge. Tells 
        Commander if Junior was told to."""
        commander_present = False
        # Update the current room
        for fiducial in sensor_input.scene.visible:
            if fiducial.type == Fiducial.TYPE_REGION:
                new_room = Entity({'EntityType' : [Predicate('EntityType',
                                                             'room')]},
                                  fiducial.id)
                self.junior_known_entities[fiducial.id] = new_room
                self.junior_known_entities[self.junior_id].predicates['Location'] = \
                    self.junior_known_entities[fiducial.id]
                break
        
        # Check if commander is present
        for fiducial in sensor_input.scene.visible:
            if fiducial.type == Fiducial.TYPE_COMMANDER:
                commander_present = True
                # Update commander's location
                if fiducial.id not in self.junior_known_entities:
                    commander_entity = \
                            Entity({'EntityType' : [Predicate('EntityType',
                                                              'Commander')],
                                    'Location' : self.junior_known_entities[self.junior_id].\
                                                    predicates['Location']},
                                   fiducial.id)
                    self.junior_known_entities[fiducial.id] = commander_entity
                    self.commander_known_entities[fiducial.id] = commander_entity
                else:
                    self.junior_known_entities[fiducial.id].predicates['Location'] = \
                            self.junior_known_entities[self.junior_id].predicates['Location']
                    self.commander_known_entities[fiducial.id].predicates['Location'] = \
                            self.junior_known_entities[self.junior_id].predicates['Location']

                break

        appeared_entities = []
        # Update knowledge with any new objects that have appeared
        for fiducial in sensor_input.scene.appeared:
            if fiducial.type != Fiducial.TYPE_COMMANDER and \
               fiducial.type != Fiducial.TYPE_REGION:
                appeared_entity = \
                        Entity({'EntityType' : [Predicate('EntityType',
                                                          fiducial.type)],
                                'Location' : Predicate('Location',
                                                        self.junior_known_entities[
                                                                self.junior_id]\
                                                            .predicates['Location'])},
                               fiducial.id)
                self.junior_known_entities[fiducial.id] = appeared_entity

                # If commander is in the room as well, update his knowledge map as well
                if commander_present:
                    self.commander_known_entities[fiducial.id] = appeared_entity

                appeared_entities.append(appeared_entity)

        # Tell commander if there is something Junior thinks he doesn't know
        self.send_event_tells(appeared_entities, commander_present)

    def send_event_tells(self, objects_appeared, commander_present):
        """Given a list of objects that the robot has just seen, check 
        if Commander is interested and notify him if he is."""
        for event in self.watch_list:
            if event.sensor == 'characterize':
                for entity in objects_appeared:
                    if (len(event.entity_class.predicates['Theme']) > 0):
                        entity_pred_values = [entitytype.value 
                                              for entitytype in entity.predicates['EntityType']]
                        if (event.entity_class.predicates['Theme'][0].value in entity_pred_values
                            and not entity.id in self.commander_known_entities):
                            object_name = event.entity_class.predicates['Theme'][0].value
                            if commander_present:
                                response = 'Do you see the %s?' % object_name
                            else:
                                response = 'I see a %s.' % object_name

                            # Put the response on Junior's discourse stack
                            self.junior_discourse_stack.append(entity)
                            self.commander_known_entities[entity.id] = entity

                            # Send info about this to the user interface
                            if self.nlmaster:
                                self.nlmaster.send_response(response)
                                print "Showing image for fiducial", entity.id, "on iPad."
                                self.map_proxy.ipad.showimage(entity.id)
                            else:
                                print response

    def get_aliased_entity(self, entity_name):
        """Checks the entity dict for a replacement name."""
        if entity_name.lower() in ENTITY_ALIASES:
            return ENTITY_ALIASES[entity_name.lower()]
        elif entity_name == 'here':
            try:
                name = self.junior_known_entities[self.junior_id].predicates['Location'].id
            except (AttributeError, KeyError):
                if self.nlmaster:
                    name = self.nlmaster.state_mgr.location
                else:
                    name = DEFAULT_LOCATION
            return name
        else:
            return entity_name

    def get_aliased_action(self, action_name):
        """Checks the action alias dict for a replacement name."""
        if action_name in ACTION_ALIASES:
            return ACTION_ALIASES[action_name]
        else:
            return action_name

    def _add_command_to_queue(self, command):
        """Converts Commands to a <(action,[arguments])> structure to add to the queue, and 
        performs aliasing for entity and action names.
        Hack: Last element in predicate list is usually most accurate."""
        action = self.get_aliased_action(command.action)
        arguments = {}
        
        # Add all predicates to arguments and add the command
        for predicate_type, predicate_list in command.entity_class.predicates.items():
            if predicate_type != 'Agent' and predicate_list:
                arguments[predicate_type] = self.get_aliased_entity(str(predicate_list[-1].value))
                      
        self.command_queue.append((action, arguments))

    def parse_semantic_structures(self, semantic_structure_list):
        """Update belief maps based on Commander's utterance, and (TODO) answer
        queries."""
        conditional = (['if'] in semantic_structure_list or ['when'] in semantic_structure_list)
        conditional_command = None
        response = ""
        
        for semantic_structure in semantic_structure_list:
            if str(semantic_structure) == "['if']" or str(semantic_structure) == "['when']":
                continue
            elif str(semantic_structure) == "or":
                continue
            else:
                self.discourse_stack.append(semantic_structure)

            # Add events to the watch list
            if isinstance(semantic_structure, Event):
                print "Adding the following to the semantics watch list:"
                print semantic_structure
                self.watch_list.append(semantic_structure)
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
                        self.junior_known_entities[new_entity.id] = new_entity
                        self.commander_known_entities[new_entity.id] = new_entity
                        
                        response += "Got it, I now know about the %s." % \
                            str(new_entity.predicates['Theme'][0].value)
                    return (self.command_queue, self.watch_list, self.commander_known_entities, response)
                else:
                    # TODO Make this actually work
                    instances = self.generate_reference_list(\
                                    semantic_structure.entity_class,
                                    self.commander_known_entities.values())

            # Answer YN questions
            if isinstance(semantic_structure, YNQuery):
                entity_matches = self.generate_reference_list(\
                                    semantic_structure.entity_class,\
                                    self.commander_known_entities.values())

                # Instant pragmatics!
                return entity_matches

            # If it's a conditional, we'll add it to the last event
            # If it's not, put it on the queue to be executed when Commander says so
            if isinstance(semantic_structure, Command):
                if conditional is True:
                    conditional_command = semantic_structure
                    conditional = False
                else:
                    self._add_command_to_queue(semantic_structure)

            # Answer WH-questions
            if isinstance(semantic_structure, WhQuery):
                answer_set = set()
                if 'Theme' in semantic_structure.entity_class.predicates:
                    theme = semantic_structure.entity_class.predicates['Theme'][0].value
                    if theme == 'you':
                        # Return Junior's position
                        if 'Location' in self.junior_known_entities[self.junior_id].predicates:
                            # Work around missing location
                            try:
                                room = str(self.junior_known_entities[self.junior_id].
                                                   predicates['Location'].id)
                                response = "I'm in the %s." % room

                                # Highlight on the ipad
                                if self.map_proxy:
                                    print "Highlighting %s on the iPad." % room
                                    self.map_proxy.ipad.highlightObject(room)
                            except AttributeError:
                                response = "I don't know."
                        else:
                            response = "I don't know."
                    elif (theme == 'he' or theme == 'she' or theme == 'it') and \
                         len(self.junior_discourse_stack) > 0:
                        
                        referent_entity = self.junior_discourse_stack[-1]

                        if referent_entity is not None and \
                           semantic_structure.predicate_type == 'Location':
                            response = str(referent_entity.predicates['Location'].value.id)

                            if self.map_proxy:
                                print "Highlighting %s on the iPad." % response
                                self.map_proxy.ipad.highlightObject(response)
                        else:
                            response = "I don't know."

                # Find the instances that satisfy the query
                    if len(answer_set) < 1: 
                        instances = self.generate_reference_list(\
                                        semantic_structure.entity_class,
                                        self.commander_known_entities.values())

                        # Generate the answer
                        if instances is not None:
                            for instance in instances:
                                if semantic_structure.predicate_type in \
                                   instance.predicates:
                                    for value in instance.predicates[semantic_structure.predicate_type]:
                                        answer_set.add(str(value))
                                        response += "The %s." % value.value

        # Add pointers to events for conditional commands
        if conditional_command is not None:
            for event in self.watch_list:
                if event.command is None:
                    event.command = conditional_command
                                               
        return (self.command_queue, self.watch_list, self.commander_known_entities, response)

    def get_quantified_list(self, quantifier, entity_references):
        """Returns a sublist of the entity references list that agrees with the given
        quantifier."""
        if len(entity_references) == 0:
            return None

        if quantifier.number is not None:
            number = int(quantifier.number)
            if number is not None:
                if number <= len(entity_references):
                    return entity_references[:number]
                else:
                    return None

        if quantifier.plural is True and quantifier.definite is True and \
           len(entity_references) < 2:
            return None

        if quantifier.plural is False:
            return [entity_references[0]]

        if quantifier.plural is True:
            return entity_references

    def get_matching_entities(self, predicate_dict, universe):
        """Get all the entities in the universe that match the predicates in
        predicate_dict. Resolve EntitySets recursively (this part doesn't work yet)."""
        resolved_result_list = []
        final_result_list = []
        
        for entity in universe:
            matches = True
            for predicate_type, predicate_value_list in predicate_dict.items():
                for predicate in predicate_value_list:
                    set_matches = False

                    # If the predicate is an EntitySet
                    if hasattr(predicate, 'quantifier'):
                        # Get the references that satisfy the predicates
                        reference_list = self.generate_reference_list(\
                                            predicate,\
                                            entity.predicates[predicate_type])
                        if reference_list is not None:
                            set_matches = True
                            break
                    else:
                        # Just find the entities that have the same predicate value
                        if predicate_type in entity.predicates and \
                           predicate in entity.predicates[predicate_type]:
                            set_matches = True
                            break

                # We want to resolve the EntitySets first
                if set_matches is False:
                    matches = False
                    break

            if matches is True:
                final_result_list.append(entity)

        for entity in resolved_result_list:
            matches = True

        return final_result_list

    def get_matching_entities_in_room(self, room_number, predicate_list, \
                                      belief_map):
        """Helper function for room-based belief maps."""
        return self.get_matching_entities(predicate_list, belief_map[room_number])

    def generate_reference_list(self, entity_class, universe):
        """Generate a list of entities (predicate dicts) corresponding to the given
        entity class in the universe."""
        entity_references = self.get_matching_entities(entity_class.predicates, \
                                                  universe)

        return self.get_quantified_list(entity_class.quantifier, \
                                        entity_references)

    def answer_query(self, query_structure):
        """Responds to a query object."""
        unknown_parameters = []
        for key, value in query_structure.parameters.items():
            if value is None:
                unknown_parameters.append(key)
            
    
def rename_entity(old_name, new_name):
    """Rename an entity."""
    # Check for double renaming by seeing whether the old_name is the value of another name
    print "Renaming", old_name, "to", new_name
    reverse_entities = dict((val, key) for key, val in ENTITY_ALIASES.items())
    if old_name in reverse_entities:
        # Change the old name to the original name
        old_name = reverse_entities[old_name]
        print "Found original name", old_name
    
    # Perform the renaming
    ENTITY_ALIASES[old_name] = new_name


if __name__ == '__main__':
    k = Knowledge()
    raw_input()
