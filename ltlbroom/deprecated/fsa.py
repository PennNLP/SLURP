"""
Defines a specific model for finite state automata, including a method to read in files
produced by JTLV and a method to execute the automaton.

Adapted from the LTLMoP toolkit.
"""

import re
import random

# Constants need to precede ltlgeneration import to prevent circular import
INIT_DEBUG = False
PRINT_SENSORS = False
ROOM_PREFIX = 'r_'
BIT_PREFIX = "b_"
ROOM_RE = re.compile(ROOM_PREFIX + '(.+)$')
BIT_ROOM_RE = re.compile(BIT_PREFIX + '(.+)$')
DONE = "done"


class FSA_State:
    """
    Each state in the automaton is an object.

    Since all states belong to a list within the Automaton object, the states may
    also be referred to just by their index within that list.  This can get confusing sometimes.
    """
    def __init__ (self, name, inputs, outputs, transitions, rank):
        # The name/rank of the state (currently the number assigned by TLV)
        self.name = name                 
        self.rank = rank   
        # NOTE: All input/output values are STRINGS.  Please cast and compare appropriately.
        # A dict of each sensor name to the value it must have to transition to this state
        self.inputs = inputs             
        # A dict of each output proposition name to the value it has in this state. Includes
        # region information
        self.outputs = outputs           
        # A list of state objects that may be transitioned to from this state
        self.transitions = transitions

    def __str__(self):
        return str(self.name)
    
    def __repr__(self):
        return "State " + self.name
    
    def info(self):
        """Return a string detailing the state."""
        return ("State: " + str(self.name) + " Rank: " + str(self.rank) + "\nInputs: " + 
                str(self.inputs) + "\nOutputs: " + str(self.outputs) + "\nSuccessors: " + 
                str(self.transitions))

###########################################################

class Automaton:
    """
    An automaton object is a collection of state objects along with information about the
    current state of the automaton when being executed. 
    """

    def __init__ (self, regions, sensor_handler, actuator_handler, motion_handler):
        """
        Creates a new automaton.

        You need to pass a list of region objects (for mapping between region names and numbers), 
        and handler objects for sensors, actuators, and region-to-region movement.
        """

        self.states = []    # A collection of state objects belonging to the automaton
        self.regions = regions

        # Store references to the handlers
        self.sensor_handler = sensor_handler
        self.actuator_handler = actuator_handler
        self.motion_handler = motion_handler

        # Variables for keeping track of the current state
        self.current_state = None
        self.current_region = None
        self.current_outputs = {}
        self.next_state = None
        self.next_region = None
        self.last_next_states = None
        
        # Automaton propositions
        self.actuators = None
        self.sensors = None
        self.custom_props = None
        
        # The generator used to create the automaton
        self.ltl_generator = None
        
        # Whether this is for following, and is thus space-unaware
        self.is_follow = None

    def stateWithName(self, name):
        """
        Find the state with the given name 
        """
        for state in self.states:
            if state.name == name:
                return state 

        raise ValueError("ERROR: Can't find state with name %s!" % (name))

    def isInitialized(self):
        """Return whether the automaton has a state selected."""
        return self.current_state is not None

    def isDone(self):
        """
        Return whether the automaton is done executing.
        """
        try:
            return self.current_outputs[DONE]
        except KeyError:
            raise ValueError("The automaton has no done output defined. "
                             "Execution will continue forever!")

    def dumpStates(self):
        """
        Print out the contents of the automaton in a human-readable format
        """
        for state in self.states:
            print "Name: ", state.name
            print "Inputs: "
            for key, val in state.inputs.iteritems():
                print key + " = " + val
            print "Outputs: "
            for key, val in state.outputs.iteritems():
                print key + " = " + val
            print "Transitions: "
            for trans in state.transitions:
                print trans.name

    def updateOutputs(self, state):
        """
        Update the values of current outputs in our execution environment to reflect the output
        proposition values associated with the given state
        """
        actuator_settings = []
        changed = False
        for key, output_val in state.outputs.items():
            # Skip any room encodings
            if ROOM_RE.match(key) or BIT_ROOM_RE.match(key): 
                continue 
            
            # Put any active actuators in the output
            output_val = (output_val == "1")
            if output_val and key in self.actuators:
                actuator_settings.append((key, output_val))

            # Note changes
            if output_val != self.current_outputs[key]:
                print "Output proposition '%s' is now %s." % (key, str(output_val))
                self.current_outputs[key] = output_val
                
                # Register the change if it's an actuator
                if key in self.actuators:
                    changed = True

        # If anything was changed, send it
        if changed:
            # Stop, in case actuation takes time
            self.motion_handler.gotoRegion(self.current_region, self.current_region)
            # Perform actuation
            self.actuator_handler.setActuators(actuator_settings)

    def regionFromState(self, state):
        """
        Given a state object, find what region is set to one.

        Also checks that multiple rooms are not set at once.
        """
        in_regions = self.ltl_generator.decode_region(state.outputs) 
        
        # Check for multiple matches
        if len(in_regions) != 1:
            raise ValueError("Found more or less than one current region: " + str(in_regions) + ". "
                             "Verify that your spec requires exactly one region at a time.")

        return in_regions[0]

    def loadFile(self, filename, sensors, actuators, custom_props, ltl_generator, is_follow):
        """
        Create an automaton by reading in a file produced by TLV.
        """

        # We will use this to decode regions
        self.ltl_generator = ltl_generator
        
        # Store whether this is follow mode
        self.is_follow = is_follow

        # These will be used later by updateOutputs() and findTransitionableState()
        self.actuators = actuators
        self.sensors = sensors
        self.custom_props = custom_props

        FILE = open(filename,"r")
        fsa_description = FILE.read()
        FILE.close()

        ###################
        # Read in states: #
        ###################

        # A magical regex to slurp up a state and its information all at once
        p = re.compile(r"State (?P<num>\d+) with rank (?P<rank>[\d\(\),-]+) -> <(?P<conds>(?:\w+:\d(?:, )?)+)>", re.IGNORECASE|re.MULTILINE)
        self.last_next_states = []
        self.next_state = None
        self.next_region = None
        m = p.finditer(fsa_description)

        # Now, for each state we find:
        for match in m:

            # Get the number (at least the number that TLV assigned the state; TLV deletes states
            # during optimization, resulting in non-consecutive numbering which would be bad for binary
            # encoding efficiency, so we don't use these numbers internally except as state names) 
            # and rank (an irrelevant synthesis byproduct that we only read in for completeness). 
            # This is the easy part.

            number = match.group('num')
            rank = match.group('rank')

            # A regex so we can iterate over "PROP = VALUE" terms
            p2 = re.compile(r"(?P<var>\w+):(?P<val>\d)", re.IGNORECASE|re.MULTILINE)
            m2 = p2.finditer(match.group('conds'))

            inputs = {}
            outputs = {} 

            # So, for each of these terms:
            for new_condition in m2:
                var = new_condition.group('var')
                val = new_condition.group('val') 
                
                # And then put it in the right place!
                if var not in sensors:
                    # If it's not a sensor proposition, then it's an output proposition
                    outputs[var] = val
                else:
                    # Oh hey it's a sensor
                    inputs[var] = val

            # We'll add transitions later; first we have to create all the states so we can
            # refer to them when we define transitions
            transitions = []

            # Create the state and add it to our collection
            newstate = FSA_State(number, inputs, outputs, transitions, rank)
            self.states.append(newstate)


        ########################
        # Read in transitions: #
        ########################

        # Another simple regex, this time for reading in transition definitions
        p = re.compile(r"State (?P<start>\d+)[^\n]+\s*With successors : (?P<ends>(?:\d+(?:, )?)+)", re.IGNORECASE|re.MULTILINE)
        m = p.finditer(fsa_description)

        # For each line:
        for match in m:
            # Get the state that the transitions come FROM
            start = match.group('start')
            # And make a list of the states that the transitions go TO
            ends = match.group('ends').split(', ')

            # Change the references to state names into references to the corresponding state objects
            ends = map(self.stateWithName, ends)

            # Stick these transitions onto the appropriate state
            self.stateWithName(start).transitions = ends

        if INIT_DEBUG:
            print "Loaded %d states." % len(self.states)
            self.dumpStates()

    def writeDot(self, filename):
        """
        Write a dot file so we can look at the automaton visually.
        """
        
        FILE = open(filename,"w")

        # Write the header
        FILE.write('digraph A { \n')
        FILE.write('\trankdir=TB;\n')
        #FILE.write('\tratio = 0.75;\n')
        FILE.write('\tsize = "8.5,11";\n')
        FILE.write('\toverlap = false;\n')
        #FILE.write('\tlayout = hierarchical;\n')

        # Write the states with region and outputs that are true
        for state in self.states:
            FILE.write('\ts'+ state.name + ' [style=\"bold\",width=0,height=0, fontsize = 20, label=\"')
            stateRegion = self.regionFromState(state)
            FILE.write(stateRegion + ' \\n ')
            for key in state.outputs.keys():
                if state.outputs[key] == '1' and not (ROOM_RE.match(key) or BIT_ROOM_RE.match(key)):
                    # Only propositions that are TRUE and not rooms are written in the state
                    FILE.write( key + ' \\n ')
            FILE.write( "("+state.rank + ')\\n ')
            FILE.write('\" ];\n')

        # Write the transitions with the input labels (only inputs that are true)
        for state in self.states:
            for nextState in state.transitions:
                FILE.write('\ts'+ state.name +' -> s'+ nextState.name +'[style=\"bold\", arrowsize = 1, fontsize = 20, label=\"')
                # Check the next state to figure out which inputs have to be on
                for key in nextState.inputs.keys():
                    if nextState.inputs[key] == '1':
                        FILE.write( key + ' \\n ')
                FILE.write('\" ];\n')    
        
        FILE.write('} \n')
        FILE.close()

    def findTransitionableStates(self, initial=False):
        """
        Returns a list of states that we could conceivably transition to, given
        the environment state (determined by querying the sensor handler)

        If ``initial`` is true, the current region and output propositions will constrain
        state selection as well. 
        """

        candidates = []

        # Define our pool of states to select from
        if initial:
            state_list = self.states
        else:
            state_list = self.current_state.transitions

        # Take a snapshot of our current sensor readings
        # This is so we don't risk the readings changing in the middle of our state search
        sensor_state = {}
        for sensor in self.sensors:
            sensor_state[sensor] = self.sensor_handler.getSensorValue(sensor)
            
        # Add in the outputs
        for output, val in self.current_outputs.items():
            sensor_state[output] = val 

        if PRINT_SENSORS:
            print "Sensor state for transition:", sensor_state

        for state in state_list:
            okay = True

            if initial:
                # First see if we can be in the state given our current region
                # Skip this check is following
                if not self.is_follow and self.regionFromState(state) != self.current_region:
                    continue

                # Now check whether our current output values match those of the state
                for key, value in state.outputs.iteritems():
                    # Ignore room propositions
                    if ROOM_RE.match(key) or BIT_ROOM_RE.match(key):
                        continue

                    if self.current_outputs[key] != (value == "1"):
                        okay = False
                        break

                if not okay:
                    continue

            # Now check whether our current sensor values match those of the state
            for key, value in state.inputs.iteritems(): 
                if int(sensor_state[key]) != int(value):                    
                    okay = False
                    break

            if okay:
                candidates.append(state)
            
        return candidates

    def chooseInitialState(self, init_region, init_outputs, skip_actuation=False):
        """
        Search through all our states to find one that satisfies our current system and environment states,
        so that we may begin our execution from there.
        
        * ``init_region`` is the number of our starting region
        * ``init_outputs`` is a list of output proposition names that are TRUE initially.
        """

        self.current_region = init_region

        for output in (self.actuators + self.custom_props):
            self.current_outputs[output] = (output in init_outputs)

        print "Matching outputs to", self.current_outputs
        candidates = self.findTransitionableStates(initial=True)

        if len(candidates) == 0: # Uh oh; that's no good
            raise ValueError("(FSA) No suitable initial state found for region %s "
                             "and sensors %s." % (init_region, str(self.current_outputs)))

        # If there's more than one candidate, let's go for variety
        self.current_state = random.choice(candidates)

        # These variables need to be cleared at the beginning of each run
        self.last_next_states = []
        self.next_state = None
        self.next_region = None

        # Bring our actuator states up-to-date
        actuator_values = []
        if not skip_actuation:
            for key, output_val in self.current_state.outputs.iteritems():
                if key in self.actuators:
                    actuator_values.append((key, output_val))

        # Send actuations if there were any. (If skip_actuation is True, this is guaranteed to be empty.)
        if actuator_values:
            self.actuator_handler.setActuators(actuator_values)

        print "Starting in state", self.current_state, "with rank", self.current_state.rank
        return self.current_state

    def runIteration(self):
        """
        Run, run, run the automaton!  (For one evaluation step)
        """
        # Return whether the state actually changed
        changed = False
        
        # Let's try to transition
        next_states = self.findTransitionableStates()

        # Make sure we have somewhere to go
        if len(next_states) == 0:
            print "None of the successor states are acceptable next states:"
            for successor in self.current_state.transitions:
                print successor.info()
            raise ValueError("(FSA) ERROR: Could not find a suitable state to transition to!")

        # Only allow self-transitions if that is the only option!
        if len(next_states) > 1 and self.current_state in next_states:
            next_states.remove(self.current_state)
        
        # See if we're beginning a new transition
        if next_states != self.last_next_states:
            # NOTE: The last_next_states comparison is also to make sure we don't
            # choose a different random next-state each time, in the case of multiple choices
            self.next_state = random.choice(next_states)
            self.next_region = self.regionFromState(self.next_state) if not self.is_follow else None
            self.last_next_states = next_states
            
            # See what we, as the system, need to do to get to this new state
            if self.next_region != self.current_region:
                ### We're going to a new region
                print "Planning to go to state", self.next_state, "with rank", self.next_state.rank
                print "Heading to region %s..." % self.next_region
                # In this case, we can't move into the next state until we've physically reached the new region
            else:
                ### The state changed, but the region didn't
                changed = self.current_state != self.next_state
                self.current_state = self.next_state  # We can transition immediately
                print "Entered:", self.next_state.info()

                # Actuate anything that might be necessary
                self.updateOutputs(self.next_state)

        # Move one step towards the next region (or stay in the same region)
        arrived = self.motion_handler.gotoRegion(self.current_region, self.next_region)

        if arrived:
            print "Crossed border from %s to %s!" % (self.current_region, self.next_region)
            print "Entered:", self.next_state.info()
            self.current_state = self.next_state
            self.current_region = self.next_region
            changed = True
            
            # Actuate anything that might be necessary
            self.updateOutputs(self.current_state)
        
        return changed
    
    def getCurrentOuputs(self):
        """Return the outputs set to 1 for the current state."""
        return [prop for prop, value in self.current_state.outputs.items() 
                if not(ROOM_RE.match(prop) or BIT_ROOM_RE.match(prop)) and value == "1"]

    def stop(self):
        """Stop all actuators and motion."""
        self.actuator_handler.stopActuators()
        self.motion_handler.gotoRegion(self.current_region, self.current_region)
