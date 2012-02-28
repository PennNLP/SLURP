#!/usr/bin/env python
"""Functions for accessing the Penn NLP Pipeline."""

import sys
import os
import re
from subprocess import Popen, PIPE

## Global constants for system configuration
# Paths
root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nlpipeline')
if not os.path.exists(root_dir):
    raise ImportError("The nlpipeline must be placed in the module directory %s "
                      "to use the Penn NLP pipeline. Run download.py to download it." % root_dir)
tool_dir = root_dir + "/tools"
parse_props = root_dir + "/models/eatb3.properties"
parse_model = root_dir + "/models/wsjall.obj"
parser_dir = root_dir + "/dbparser-0.9.9c-modified"
ecrestore_dir = root_dir + "/addnulls-mod" 

# Parser/java options
java = "java"
max_heap = "1000"
parser_class = "danbikel.parser.Parser"
settings = "-Dparser.settingsFile=%s" % parse_props
parser_classpath = parser_dir + "/dbparser.jar:" + parser_dir + "/dbparser-ext.jar"

# EC Restore/java options
ecrestore_classpath = "%s:%s/mallet-0.4/class" % (ecrestore_dir, tool_dir)

# Pipeline commands
SED = "sed -l" if sys.platform == "darwin" else "sed -u"
tokenizer = "%s -f %s/tokenizer.sed" % (SED, tool_dir)
tagger = "java -Xmx128m -classpath %s/mxpost/mxpost.jar tagger.TestTagger %s/mxpost/tagger.project/ 2> /dev/null" % (root_dir, root_dir)
parser = "%s -Xms%sm -Xmx%sm -cp %s %s %s -is %s  -sa - -out - 2> /dev/null" % \
    (java, max_heap, max_heap, parser_classpath, settings, parser_class, parse_model)
ecrestore_wrapper = "%s/wrap-stream.pl" % ecrestore_dir 
ecrestorer = "%s -cp %s edu.upenn.cis.emptycategories.RestoreECs run - --perceptron --ante_perceptron --nptrace --whxp --wh --whxpdiscern --nptraceante --noante 2> /dev/null" \
    % (java, ecrestore_classpath)

tokentag_command = " | ".join((tokenizer, tagger))
parser_command = parser
ecrestore_command = " | ".join((ecrestore_wrapper, ecrestorer))

tokentag_pipe = None
parse_pipe = None
ecrestore_pipe = None

OUTER_PARENS_RE = re.compile("\(\s*(.+)\s*\)")


def init_pipes():
    """Initialize the nl processing pipelines.

    You must call this before calling any other functions"""
    # pylint: disable=W0603
    global tokentag_pipe, parse_pipe, ecrestore_pipe

    # Set up the pipes
    tokentag_pipe = setup_pipe(tokentag_command)
    parse_pipe = setup_pipe(parser_command)
    ecrestore_pipe = setup_pipe(ecrestore_command, ecrestore_dir)


def close_pipes():
    """Terminate all pipelines."""
    tokentag_pipe.terminate()
    parse_pipe.terminate()
    ecrestore_pipe.terminate()


def setup_pipe(command, cwd=None):
    """Set up a pipeline using the given command."""
    return Popen(command, stdin=PIPE, stdout=PIPE, shell=True, cwd=cwd)


def process_pipe_filter(text, process, line_filter=""):
    """Run text through the pipe, returning the first output line starting with the filter."""
    print >> process.stdin, text
    process.stdin.flush()
    if filter:
        text = process.stdout.readline().strip()
        while not text.startswith(line_filter):
            text = process.stdout.readline().strip()
    else: 
        text = process.stdout.readline().strip()

    return text


def parse_text(sent, force_nouns=set(), force_verbs=set()):
    """Run the text through the pipelines."""
    if not tokentag_pipe or not parse_pipe or not ecrestore_pipe:
        raise ValueError("You must call init_pipes before parsing")

    tagged_sent = process_pipe_filter(sent, tokentag_pipe)
    bikel_clean_tagged = _tag_convert(tagged_sent, force_nouns, force_verbs)
    parsed_sent = process_pipe_filter(bikel_clean_tagged, parse_pipe, "(")
    restored_sent = process_pipe_filter(parsed_sent, ecrestore_pipe, "(")

    # Remove extra parens from the parse with elements restored
    final_parse = OUTER_PARENS_RE.match(restored_sent).group(1)

    return final_parse


def _tag_convert(sent, force_nouns, force_verbs):
    """Convert from MXPOST to Bikel style tags, coercing tags."""
    # MXPOST: word_tag
    token_tags = [token.split('_') for token in sent.rstrip().split()]
    # Coerce the tags
    token_tags = [(word, _coerce_tag(word, tag, force_nouns, force_verbs)) 
                  for word, tag in token_tags]
    # Bikel: (word (tag)), with () around the line
    return "(" + " ".join("(%s (%s))" % (word, tag) for word, tag in token_tags) + ")"


def _coerce_tag(word, tag, force_nouns, force_verbs):
    """Coerce a word's tag if it's in the sets force_nouns or force_verbs."""
    tag = "NN" if word.lower() in force_nouns else tag
    tag = "VB" if word.lower() in force_verbs else tag
    return tag


if __name__ == "__main__":
    print "Pipeline paths:"
    print tokenizer
    print tagger
    print parser
    print ecrestore_wrapper
    print ecrestorer

