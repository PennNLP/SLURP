#!/usr/bin/env python
"""Functions for accessing the Penn NLP Pipeline.

Depends on the nlpipeline, sed, and java."""

import sys
import os
import re
from subprocess import Popen, PIPE
import shlex

## Global constants for system configuration
# Paths
NULL = "NUL" if sys.platform == "win32" else "/dev/null"
CLASSPATH_SEP = ";" if sys.platform == "win32" else ":"
root_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nlpipeline')
if not os.path.exists(root_dir):
    raise ImportError("The nlpipeline must be placed in the module directory %s "
                      "to use the Penn NLP pipeline. Run download.py to download it." % root_dir)
tool_dir = os.path.join(root_dir, "tools")
parse_props = os.path.join(root_dir, "models", "eatb3.properties")
parse_model = os.path.join(root_dir, "models", "wsjall.obj")
parser_dir = os.path.join(root_dir, "dbparser-0.9.9c-modified")
ecrestore_dir = os.path.join(root_dir, "addnulls-mod") 

# Parser/java options
java = "java"
parser_class = "danbikel.parser.Parser"
settings = "-Dparser.settingsFile=%s" % parse_props
parser_classpath = (os.path.join(parser_dir, "dbparser.jar") + CLASSPATH_SEP + 
                    os.path.join(parser_dir, "dbparser-ext.jar"))

# EC Restore/java options
ecrestore_classpath = ecrestore_dir + CLASSPATH_SEP + os.path.join(tool_dir, "mallet-0.4", "class")

# Pipeline commands
SED = "sed -l" if sys.platform == "darwin" else "sed -u"
tokenizer = SED +" -f " + os.path.join(tool_dir, "tokenizer.sed")
tagger_jar = os.path.join(root_dir, "mxpost", "mxpost.jar")
tagger_project = os.path.join(root_dir, "mxpost", "tagger.project")
tagger = "java -Xmx128m -classpath %s tagger.TestTagger %s" % (tagger_jar, tagger_project)
parser = "%s -Xmx768m -cp %s %s %s -is %s  -sa - -out -" % \
    (java, parser_classpath, settings, parser_class, parse_model)
ecrestorer = "%s -cp %s edu.upenn.cis.emptycategories.RestoreECs run - --perceptron --ante_perceptron --nptrace --whxp --wh --whxpdiscern --nptraceante --noante" \
    % (java, ecrestore_classpath)

token_proc = None
tag_proc = None
parse_proc = None
ecrestore_proc = None
null_out = None

OUTER_PARENS_RE = re.compile("\(\s*(.+)\s*\)")


def init_pipes():
    """Initialize the nl processing pipelines.

    You must call this before calling any other functions"""
    # pylint: disable=W0603
    global token_proc, tag_proc, parse_proc, ecrestore_proc, null_out

    # Set up the null sink and the pipes
    null_out = open(NULL, 'w')
    token_proc = setup_pipe(tokenizer)
    tag_proc = setup_pipe(tagger)
    parse_proc = setup_pipe(parser)
    ecrestore_proc = setup_pipe(ecrestorer, ecrestore_dir)


def close_pipes():
    """Terminate all pipelines."""
    token_proc.terminate()
    tag_proc.terminate()
    parse_proc.terminate()
    ecrestore_proc.terminate()
    null_out.close()


def setup_pipe(command, cwd=None):
    """Set up a pipeline using the given command."""
    # Windows expects a string for the command, so only lex other systems
    if sys.platform != "win32":
        command = shlex.split(command)
    
    try:
        return Popen(command, stdin=PIPE, stdout=PIPE, stderr=null_out, cwd=cwd)
    except OSError:
        raise OSError("Subprocess failed to run command: %s" % command)


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


def parse_text(text, force_nouns=set(), force_verbs=set()):
    """Run the text through the pipelines."""
    if not all((token_proc, tag_proc, parse_proc, ecrestore_proc)):
        raise ValueError("You must call init_pipes before parsing.")

    token_sent = process_pipe_filter(text, token_proc)
    tagged_sent = process_pipe_filter(token_sent, tag_proc)
    clean_tagged_sent = _tag_convert(tagged_sent, force_nouns, force_verbs)
    parsed_sent = process_pipe_filter(clean_tagged_sent, parse_proc, "(")
    # Wrap input to the null restorer as ( x) exactly as wrap-stream.pl used to do
    restored_sent = process_pipe_filter("( " + parsed_sent + ")", ecrestore_proc, "(")

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
    print ecrestorer

