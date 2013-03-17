#!/usr/bin/env python
"""Demonstration of the Penn NLP pipeline and semantics processing."""

import sys
import _curses
import curses.textpad
import signal

from pennpipeline import PennPipeline
from semantics import knowledge, tree

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

    # Calculate window heights, top 1/4 is input, middle half is parse, remainder is semantics 
    input_height = screen_height / 6
    parse_height = screen_height / 6
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
    input_frame.refresh() # Extra refresh to grab focus
      
    return input_frame, input_win, parse_win, semantic_win


def get_input(input_win):
    """Get user input from a window."""
    input_win.erase()
    input_win.refresh()
    return curses.textpad.Textbox(input_win, insert_mode=True).edit().rstrip()


def interactive_mode(window):
    """Interactively get input from the user and parse it."""
    input_frame, input_win, parse_win, semantic_win = setup_windows(window)

    # Initialize pipeline
    pipeline = PennPipeline()

    # Set up semantics module
    world_knowledge = knowledge.Knowledge()
    
    # Send some data through the pipeline
    result = pipeline.parse_text("This is a test.")
    input_frame.addstr(1, 1, 'Enter your input, then press Ctrl+G. '
                       'Enter "quit" or press Ctrl+C to exit.')
    input_frame.refresh()

    # Until the input is q/quit, process data
    while True:
        # Get text from the input box, removing any embedded newlines
        text = get_input(input_win).strip()

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
        result = pipeline.parse_text(text)

        # Clear the status and output the result, it's easiest to just
        # clear and echo the input again 
        parse_win.clear()
        try:
            parse_win.addstr(text + '\n')
            parse_win.addstr(result)
        except _curses.error:
            parse_win.clear()
            semantic_win.addstr("Parse too large to show.""")
        parse_win.refresh()   

        # Do the same for semantics
        # Echo input and display status, after clearing the window
        semantic_win.clear()
        semantic_win.addstr(text)
        semantic_win.addstr('\nPerforming semantic analysis...')
        
        semantic_win.refresh()

        process_result = world_knowledge.process_parse_tree(result, text)
        semantic_answer, frame_trees = process_result[0], process_result[1]
        if frame_trees is not None:
            modified_trees = [str(modified_parse_tree[1]) for modified_parse_tree in frame_trees 
                              if (len(modified_parse_tree) > 1 and 
                                  isinstance(modified_parse_tree[1], tree.Tree))]
            # TODO: Remove after dupes stop coming in
            modified_trees = list(set(modified_trees))
            frames = [str(frame_dict) for frame_dict in [frame[0] for frame in frame_trees \
                                                         if not isinstance(frame, str)]] 
            semantics = "Modified trees: " + "\n".join(modified_trees) + \
                        "\nFrames: " + "\n".join(frames) + "\nResponse: " + str(semantic_answer)
        else:
            semantics = str(semantic_answer)

        # Clear the status and output the result, it's easiest to just
        # clear and echo the input again 
        semantic_win.clear()
        try:
            semantic_win.addstr(text + '\n')
            semantic_win.addstr(semantics)
        except _curses.error:
            semantic_win.clear()
            semantic_win.addstr("Semantics representation too large to show.""")
        semantic_win.refresh()     

    return


def main():
    """Get input and process it in a loop."""

    # Get input from user interactively    
    signal.signal(signal.SIGWINCH, sigwinch_handler)
    try:
        curses.wrapper(interactive_mode)
    except WindowTooSmallError as exc:
        print >> sys.stderr, exc
        sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        # This is just to handle ctrl+c gracefully without a callstack
        pass
