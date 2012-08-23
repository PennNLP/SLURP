
import parsing
from lexical_constants import *

def process_parse_tree(self, parse_tree_input, text_input):
    """Produces semantic interpretations of parse trees."""

    print "Processing:", repr(text_input)
    semantics_result = parsing.get_semantics_from_parse_tree(parse_tree_input)
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
