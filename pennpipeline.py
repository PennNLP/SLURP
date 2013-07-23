#!/usr/bin/env python
"""Functions for accessing the SUBTLE MURI NLP Pipeline.

Depends on the SUBTLE Pipeline and Java."""

import sys
import os
import re
from subprocess import Popen, PIPE, check_call
import shlex


## Global constants for configuration
# Current version, which must match the downloaded pipeline
CURRENT_VERSION = "1.1.2"
# Paths
PIPELINE_NAME = "SUBTLEPipeline-master"
CLASSPATH_SEP = ";" if sys.platform == "win32" else ":"
JAVA = "java"
TEST_COMMAND = "{} -version".format(JAVA)
ROOT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), PIPELINE_NAME)

# Check for up-to-date pipeline
if not os.path.exists(ROOT_DIR):
    raise ImportError("The folder for the SUBTLE Pipeline %s does not exist. "
                      "Run download.py in the root of the SLURP repository to automatically "
                      "download and decompress the current version." %  ROOT_DIR)
PIPELINE_VERSION = "unknown"
try:
    PIPELINE_VERSION = open(os.path.join(ROOT_DIR, "VERSION")).read().strip()
    if PIPELINE_VERSION != CURRENT_VERSION:
        raise IOError
except IOError:
    raise ImportError("The copy of the SUBTLE Pipeline in %s (version %s) is not the same version "
                      "as pennpipeline.py (version %s). You should:\n"
                      "1. Make sure you have the latest updates from the PennNLP/SLURP GitHub "
                      "repository.\n"
                      "2.Run download.py in the root of the SLURP repository "
                      "to update the SUBTLE Pipeline." % (ROOT_DIR, PIPELINE_VERSION, CURRENT_VERSION))

# Tagger
TAGGER_NAME = "stanford-postagger-streaminterface"
TAGGER_JAR = os.path.join(ROOT_DIR, TAGGER_NAME, TAGGER_NAME + ".jar")
TAGGER_MODEL = os.path.join(ROOT_DIR, TAGGER_NAME, "models", "english-caseless-left3words-distsim.tagger")

# Parser
PARSE_PROPS = os.path.join(ROOT_DIR, "models", "eatb3.properties")
PARSE_MODEL = os.path.join(ROOT_DIR, "models", "wsjall.obj")
PARSER_DIR = os.path.join(ROOT_DIR, "dbparser-0.9.9c-modified")
PARSER_CLASS = "danbikel.parser.Parser"
PARSER_SETTINGS = '-Dparser.settingsFile="%s"' % PARSE_PROPS
PARSER_CLASSPATH = (os.path.join(PARSER_DIR, "dbparser.jar") + CLASSPATH_SEP +
                    os.path.join(PARSER_DIR, "dbparser-ext.jar"))

# EC Restorer
ECRESTORER_DIR = os.path.join(ROOT_DIR, "restore-ecs")
ECRESTORER_JAR = os.path.join(ECRESTORER_DIR, "restore-ecs.jar")

# Pipeline commands
TAGGER = '%s -Xmx256m -jar "%s" "%s"' % (JAVA, TAGGER_JAR, TAGGER_MODEL)
PARSER = '%s -Xmx1024m -cp "%s" %s %s -is "%s" -sa - -out -' % \
    (JAVA, PARSER_CLASSPATH, PARSER_SETTINGS, PARSER_CLASS, PARSE_MODEL)
ECRESTORER = ('%s -Xmx256m -jar "%s" '
              "run - --perceptron --ante_perceptron") % (JAVA, ECRESTORER_JAR)

# Pipeline constants
OUTER_PARENS_RE = re.compile(r"\(\s*(.+)\s*\)")


class PennPipeline(object):
    """Provide access to a pipeline of NLP tools."""

    def __init__(self):
        # Set up the pipes
        self.tag_proc, self.parse_proc, self.ecrestore_proc = (None, None, None)
        self.tag_proc = _setup_pipe(TAGGER)
        self.parse_proc = _setup_pipe(PARSER)
        self.ecrestore_proc = _setup_pipe(ECRESTORER, ECRESTORER_DIR)

    def __del__(self):
        """Terminate all pipelines."""
        # This is possibly unnecessary, but doesn't do any harm.
        for proc in (self.tag_proc, self.parse_proc, self.ecrestore_proc):
            _terminate_with_extreme_prejudice(proc)

    def parse_text(self, text, force_nouns=None, force_verbs=None, correct_punc=True,
                   verbose=False):
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

        # TODO: Deal with multiple sentences
        try:
            tagged_sent = _process_pipe_filter(text, self.tag_proc, read_until_empty=True)
        except IOError:
            raise IOError("Tagger process has died. Make sure that the command\n{}\ncan "
                          "be run successfully.".format(TAGGER))
        if verbose:
            # Show the tags without any forced tags
            display_tagged_sent = _tag_convert(tagged_sent, set(), set())
            print "Original tags:"
            print display_tagged_sent
        clean_tagged_sent = _tag_convert(tagged_sent, force_nouns, force_verbs)
        if verbose and (force_nouns or force_verbs) and (clean_tagged_sent != display_tagged_sent):
            print "Modified tags:"
            print clean_tagged_sent
        try:
            parsed_sent = _process_pipe_filter(clean_tagged_sent, self.parse_proc, "(")
        except IOError:
            raise IOError("Parser process has died. Make sure that the command\n{}\ncan "
                          "be run successfully.".format(PARSER))
        # Wrap input to the null restorer as ( x) exactly as wrap-stream.pl used to do
        try:
            restored_sent = _process_pipe_filter("( " + parsed_sent + ")", self.ecrestore_proc, "(")
        except IOError:
            raise IOError("ECRestorer process has died. Make sure that the command\n{}\ncan "
                          "be run successfully in the {} directory.".format(ECRESTORER, ECRESTORER_DIR))

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
        print >> sys.stderr, "Subprocess failed to run command: %s" % command
        raise


def _process_pipe_filter(text, process, line_filter="", read_until_empty=False):
    """Run text through the pipe, returning the first output line starting with the filter."""
    print >> process.stdin, text
    process.stdin.flush()
    if filter:
        text = process.stdout.readline().strip()
        while not text.startswith(line_filter):
            text = process.stdout.readline().strip()
    else:
        text = process.stdout.readline().strip()

    # Read additional lines until we get a blank one
    if read_until_empty:
        while True:
            if not process.stdout.readline().strip():
                break

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
    print "Tagger:"
    print TAGGER
    print "Parser:"
    print PARSER
    print "EC Restorer:"
    print ECRESTORER
    print

    # Try to run dependencies
    print "Trying to run dependencies..."
    print "Testing {!r}...".format(JAVA)
    check_call(shlex.split(TEST_COMMAND) if sys.platform != "win32" else TEST_COMMAND)
