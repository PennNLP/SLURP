#!/usr/bin/env python
"""Functions for accessing the Penn NLP Pipeline."""

import os
import re
from subprocess import Popen, PIPE

## Global constants for system configuration
# Paths
root_dir = os.path.expanduser("~/nlpipeline")
if not os.path.exists(root_dir):
    raise ImportError("The nlpipeline must be placed in the home directory "
                      "to use the Penn NLP pipeline.")
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
tokenizer = "sed -u -f %s/tokenizer.sed" % tool_dir
tagger = "java -Xmx128m -classpath %s/mxpost/mxpost.jar tagger.TestTagger %s/mxpost/tagger.project/ 2> /dev/null" % (root_dir, root_dir)
tagger_post = "%s/adwait2bikel.pl" % tool_dir
parser = "%s -Xms%sm -Xmx%sm -cp %s %s %s -is %s  -sa - -out - 2> /dev/null" % \
    (java, max_heap, max_heap, parser_classpath, settings, parser_class, parse_model)
ecrestore_wrapper = "%s/wrap-stream.pl" % ecrestore_dir 
ecrestorer = "%s -cp %s edu.upenn.cis.emptycategories.RestoreECs run - --perceptron --ante_perceptron --nptrace --whxp --wh --whxpdiscern --nptraceante --noante 2> /dev/null" \
    % (java, ecrestore_classpath)

tokentag_command = " | ".join((tokenizer, tagger))
parser_command = " | ".join((tagger_post, parser))
ecrestore_command = " | ".join((ecrestore_wrapper, ecrestorer))

tokentag_pipe = None
parse_pipe = None
ecrestore_pipe = None


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


OUTER_PARENS_RE = re.compile("\(\s*(.+)\s*\)")
def parse_text(sent):
    """Run the text through the pipelines."""
    if not tokentag_pipe or not parse_pipe or not ecrestore_pipe:
        raise ValueError("You must call init_pipes before parsing")

    tagged_sent = process_pipe_filter(sent, tokentag_pipe)
    parsed_sent = process_pipe_filter(tagged_sent, parse_pipe, "(")
    restored_sent = process_pipe_filter(parsed_sent, ecrestore_pipe, "(")

    # Remove extra parens from the parse with elements restored
    final_parse = OUTER_PARENS_RE.match(restored_sent).group(1)

    return final_parse


if __name__ == "__main__":
    print "Pipeline paths:"
    print tokenizer
    print tagger
    print tagger_post
    print parser
    print ecrestore_wrapper
    print ecrestorer
