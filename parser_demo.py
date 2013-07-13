#!/usr/bin/env python
"""Demonstration of the Penn NLP pipeline and semantics processing."""

import sys
import _curses
import curses.textpad
import signal

from pipelinehost import PipelineClient
from semantics.parsing import process_parse_tree
from semantics.tree import Tree
from semantics.new_knowledge import KnowledgeBase

MIN_HEIGHT = 35


class WindowTooSmallError(RuntimeError):
    """Error for when the terminal window is too small for the UI."""
    pass


def sigwinch_handler(dummy, unused):
    """Handle SIGWINCH events by recreating the window.

    This avoids crashes, but doesn't actually resize the UI."""
    curses.endwin()
    curses.initscr()


def setup_windows(master_window):
    """Get input from the user using a Textbox."""
    # Get the window size
    (screen_height, screen_width) = master_window.getmaxyx()
    if screen_height < MIN_HEIGHT:
        raise WindowTooSmallError("The window is too small. " +
                                  "Resize to at least %d lines " % MIN_HEIGHT +
                                  "and try again.")

    # Calculate window heights
    input_height = screen_height / 6
    parse_height = (screen_height - input_height) / 2
    semantic_height = screen_height - input_height - parse_height

    # Build input frame
    input_frame = master_window.subwin(input_height, screen_width, 0, 0)
    input_frame.border()
    input_frame.addstr(0, 1, "Input")
    input_frame.addstr(1, 1, 'Waiting for parsing pipeline to load...')

    # Make an input window just inside the outer frame
    input_win = input_frame.derwin(input_height - 3, screen_width - 2, 2, 1)
    input_frame.refresh()

    # Create the parse output frame and box similarly
    parse_frame = master_window.derwin(parse_height, screen_width, input_height, 0)
    parse_frame.border()
    parse_frame.addstr(0, 1, "Parse")
    parse_frame.refresh()
    parse_win = parse_frame.derwin(parse_height - 2, screen_width - 2, 1, 1)
    parse_win.refresh()

    # And once again for semantics
    semantic_frame = master_window.derwin(semantic_height, screen_width,
                                          input_height + parse_height, 0)
    semantic_frame.border()
    semantic_frame.addstr(0, 1, "Semantics")
    semantic_frame.refresh()
    semantic_win = semantic_frame.derwin(semantic_height - 2, screen_width - 2,
                                         1, 1)
    semantic_win.refresh()
    input_frame.refresh()  # Extra refresh to grab focus

    return input_frame, input_win, parse_win, semantic_win


def get_input(input_win):
    """Get user input from a window."""
    return curses.textpad.Textbox(input_win, insert_mode=True).edit().rstrip()


def interactive_mode(window, first_input):
    """Interactively get input from the user and parse it."""
    input_frame, input_win, parse_win, semantic_win = setup_windows(window)

    # Initialize pipeline and knowledge base
    pipeline = PipelineClient()
    kb = KnowledgeBase()

    # Send some data through the pipeline
    result = pipeline.parse("This is a test.")
    input_frame.addstr(1, 1, 'Enter your input, then press Ctrl+G. '
                       'Enter "quit" or press Ctrl+C to exit.')
    input_frame.refresh()

    # Until the input is q/quit, process data
    last_input = first_input
    while True:
        # Display the first input if needed
        input_win.erase()
        input_win.refresh()
        if last_input:
            input_win.addstr(0, 0, last_input)

        # Get text from the input box, removing any embedded newlines
        if first_input:
            text = first_input
            first_input = None
        else:
            text = get_input(input_win).replace("\n", "").strip()
        last_input = text

        # Quit if needed
        if text == "q" or text == "quit":
            return

        # Get input again if it was empty
        if not text:
            continue

        # Echo input and display status, clearing both windows
        parse_win.clear()
        parse_win.addstr(text)
        parse_win.addstr('\nParsing and restoring null elements...')
        parse_win.refresh()
        semantic_win.clear()
        semantic_win.refresh()

        # Run the parse pipeline
        result = pipeline.parse(text)
        result_tree = Tree(result)

        # Output the longest parse that will fit. We try to draw the
        # possible output in order of decreasing length.
        parse_max_width = parse_win.getmaxyx()[1]
        possible_formats = (result_tree.pprint(margin=parse_max_width, force_multiline=True),
                            result_tree.pprint(margin=parse_max_width),
                            result)

        for formatted_result in possible_formats:
            parse_win.clear()
            try:
                parse_win.addstr(text + '\n')
                parse_win.addstr(formatted_result)
            except _curses.error:
                continue
            else:
                # We've successfully printed, stop trying formats
                break
        else:
            parse_win.clear()
            parse_win.addstr("Parse too large to show.\n")
        parse_win.refresh()

        # Do the same for semantics
        # Echo input and display status, after clearing the window
        semantic_win.clear()
        semantic_win.addstr(text)
        semantic_win.addstr('\nPerforming semantic analysis...')
        semantic_win.refresh()

        frames, new_commands, kb_response = process_parse_tree(result, text, kb)
        semantic_win.clear()
        try:
            if frames:
                semantic_win.addstr("Frames matched:\n")
                for frame in frames:
                    semantic_win.addstr("\t" + str(frame) + "\n")
            if new_commands:
                semantic_win.addstr("New commands:\n")
                for command in new_commands:
                    semantic_win.addstr(str(command) + "\n")
            if kb_response:
                semantic_win.addstr("KB response:\n")
                semantic_win.addstr(str(kb_response) + "\n")
            if not any((frames, new_commands, kb_response)):
                semantic_win.addstr("No frames matched.\n")
        except _curses.error:
            semantic_win.clear()
            semantic_win.addstr("Semantic representation too large to show.")
        semantic_win.refresh()

    return


def main(first_input=None):
    """Get input and process it in a loop."""

    # Get input from user interactively
    signal.signal(signal.SIGWINCH, sigwinch_handler)
    try:
        curses.wrapper(interactive_mode, first_input)
    except WindowTooSmallError as exc:
        print >> sys.stderr, exc
        sys.exit(1)


if __name__ == "__main__":
    try:
        # If there's one argument, pass it as the first input string
        main(sys.argv[1] if len(sys.argv) == 2 else None)
    except KeyboardInterrupt:
        # This is just to handle ctrl+c gracefully without a callstack
        pass
