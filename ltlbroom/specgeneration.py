"""Generates a logical specification from natural language."""


def generate(text, sensors, regions, props):
    """Generate a logical specification from natural language and propositions.""" 
    responses = []
    for line in text.split('\n'):
        print "Parsing:", repr(line)
        responses.append("Ok!")
        #parse = parse_text(text)
        #result = WORLD_KNOWLEDGE.process_parse_tree(parse, text)
    
    environment_lines = []
    system_lines = ["[]<>(s.%s)" % region for region in regions]
    custom_props = []

    print "Spec generation complete."    
    return environment_lines, system_lines, custom_props
