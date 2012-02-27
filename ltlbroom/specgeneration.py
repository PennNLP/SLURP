"""Generates a logical specification from natural language."""


def generate(text, sensors, regions, props):
    """Generate a logical specification from natural language and propositions."""
    print "NL->LTL Generation called on:"
    print "Sensors:", sensors
    print "Props:", props
    print "Regions:", regions

    responses = []
    for line in text.split('\n'):
        if not line:
            continue

        # Clean unicode first if needed
        if isinstance(line, unicode):
            line = line.encode('ascii', 'ignore')
        
        print "Parsing:", repr(line)
        responses.append("Ok!")
        #parse = parse_text(text)
        #result = WORLD_KNOWLEDGE.process_parse_tree(parse, text)
    
    environment_lines = []
    system_lines = ["[]<>(s.%s)" % region for region in regions]
    custom_props = []

    print "Spec generation complete."    
    return environment_lines, system_lines, custom_props
