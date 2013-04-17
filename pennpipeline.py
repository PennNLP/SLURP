#!/usr/bin/env python
"""Functions for accessing the Penn NLP Pipeline.

Depends on the nlpipeline, sed, and Java."""

import sys
import os
import re
from subprocess import Popen, PIPE
import shlex

## Global constants for system configuration
# Paths
CLASSPATH_SEP = ";" if sys.platform == "win32" else ":"
ROOT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'nlpipeline')
if not os.path.exists(ROOT_DIR):
    raise ImportError("The nlpipeline must be placed in the module directory %s "
                      "to use the Penn NLP pipeline. Run download.py to download it." % ROOT_DIR)
TOOL_DIR = os.path.join(ROOT_DIR, "tools")
PARSE_PROPS = os.path.join(ROOT_DIR, "models", "eatb3.properties")
PARSE_MODEL = os.path.join(ROOT_DIR, "models", "wsjall.obj")
PARSER_DIR = os.path.join(ROOT_DIR, "dbparser-0.9.9c-modified")
ECRESTORE_DIR = os.path.join(ROOT_DIR, "addnulls-mod")

# Parser/JAVA options
JAVA = "java"
PARSER_CLASS = "danbikel.parser.Parser"
PARSER_SETTINGS = "-Dparser.settingsFile=%s" % PARSE_PROPS
PARSER_CLASSPATH = (os.path.join(PARSER_DIR, "dbparser.jar") + CLASSPATH_SEP +
                    os.path.join(PARSER_DIR, "dbparser-ext.jar"))

# EC Restore/JAVA options
ECRESTORER_CLASSPATH = ECRESTORE_DIR + CLASSPATH_SEP + os.path.join(TOOL_DIR, "mallet-0.4", "class")

# Pipeline commands
SED = "sed -l" if sys.platform == "darwin" else "sed -u"
TOKENIZER = SED + " -f " + os.path.join(TOOL_DIR, "tokenizer.sed")
TAGGER_JAR = os.path.join(ROOT_DIR, "mxpost", "mxpost.jar")
TAGGER_PROJECT = os.path.join(ROOT_DIR, "mxpost", "tagger.project")
TAGGER = "%s -Xmx128m -classpath %s tagger.TestTagger %s" % (JAVA, TAGGER_JAR, TAGGER_PROJECT)
PARSER = "%s -Xmx1024m -cp %s %s %s -is %s  -sa - -out -" % \
    (JAVA, PARSER_CLASSPATH, PARSER_SETTINGS, PARSER_CLASS, PARSE_MODEL)
ECRESTORER = ("%s -Xmx256m -cp %s edu.upenn.cis.emptycategories.RestoreECs "
              "run - --perceptron --ante_perceptron --nptrace --whxp --wh "
              "--whxpdiscern --nptraceante --noante") % (JAVA, ECRESTORER_CLASSPATH)

# Pipeline constants
OUTER_PARENS_RE = re.compile(r"\(\s*(.+)\s*\)")


class PennPipeline(object):
    """Provide access to a pipeline of NLP tools."""

    def __init__(self):
        # Set up the pipes
        self.token_proc = _setup_pipe(TOKENIZER)
        self.tag_proc = _setup_pipe(TAGGER)
        self.parse_proc = _setup_pipe(PARSER)
        self.ecrestore_proc = _setup_pipe(ECRESTORER, ECRESTORE_DIR)

    def __del__(self):
        """Terminate all pipelines."""
        # This is possibly unnecessary, but doesn't do any harm.
        for proc in (self.token_proc, self.tag_proc, self.parse_proc, self.ecrestore_proc):
            _terminate_with_extreme_prejudice(proc)

    def parse_text(self, text, force_nouns=None, force_verbs=None, correct_punc=True):
        """Run the text through the pipelines."""
        # Check for empty text
        if not text:
            return ""

        # Replace any newlines or carriage returns with space
        text = text.replace('\n', ' ')
        text = text.replace('\r', ' ')

        # Create default set arguments
        if not force_nouns:
            force_nouns = set()
        if not force_verbs:
            force_verbs = set()

        # Add in final punctuation if it's missing
        if correct_punc and text[-1] not in ('.', '?', '!'):
            text += '.'

        token_sent = _process_pipe_filter(text, self.token_proc)
        tagged_sent = _process_pipe_filter(token_sent, self.tag_proc)
        clean_tagged_sent = _tag_convert(tagged_sent, force_nouns, force_verbs)
        parsed_sent = _process_pipe_filter(clean_tagged_sent, self.parse_proc, "(")
        # Wrap input to the null restorer as ( x) exactly as wrap-stream.pl used to do
        restored_sent = _process_pipe_filter("( " + parsed_sent + ")", self.ecrestore_proc, "(")

        # Remove extra parens from the parse with elements restored
        final_parse = OUTER_PARENS_RE.match(restored_sent).group(1)

        return final_parse


def _terminate_with_extreme_prejudice(proc):
    """Terminate a process without any regard for exceptions."""
    try:
        proc.terminate()
    # pylint: disable=W0702
    except:
        pass


def _setup_pipe(command, cwd=None):
    """Set up a pipeline using the given command."""
    # Windows expects a string for the command, so only lex other systems
    if sys.platform != "win32":
        command = shlex.split(command)

    try:
        return Popen(command, stdin=PIPE, stdout=PIPE, stderr=open(os.devnull, 'w'), cwd=cwd)
    except OSError:
        raise OSError("Subprocess failed to run command: %s" % command)


def _process_pipe_filter(text, process, line_filter=""):
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


def _tag_convert(sent, force_nouns, force_verbs):
    """Convert from MXPOST to Bikel style tags, coercing tags."""
    # MXPOST: word_tag
    # We use rsplit with one to make sure underscores in the token aren't harmed.
    token_tags = [token.rsplit('_', 1) for token in sent.rstrip().split()]
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
    print TOKENIZER
    print TAGGER
    print PARSER
    print ECRESTORER
