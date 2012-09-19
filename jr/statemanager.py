#!/usr/bin/python
"""Coordinate state between language, automata, and the robot."""

import re
from threading import Lock

from semantics.lexical_constants import (SEARCH_ACTION, GO_ACTION, GET_ACTION, FOLLOW_ACTION, 
                                 SEE_ACTION, TELL_ACTION, ACTION_ALIASES)
from ltlbroom.fsa import Automaton
from ltlbroom.ltlgeneration import LTLGenerator, FOLLOW_PROP, SEARCH_PROP, EXPLORE_PROP, UnrealizableError
from nlproxy import send_ltl_commands
from worldmap import get_adjacent_rooms, adjacency_changed
from subtle_msgs.msg import Fiducial

# LTL constants
ACTION_RE = re.compile(r"(?P<action>[a-z]+)(_(?P<target>[a-z0-9]+))?")
DRIVE_COMMAND = "drive_to"
DEFUSE_PROP = "defuse"
COMPLEX_ACTUATORS = {"get": "get_obj"}
SENSOR_MAPPING = {Fiducial.TYPE_BOMB: "bomb",
                  Fiducial.TYPE_HOSTAGE: "hostage",
                  Fiducial.TYPE_BADGUY: "bad_guy",
                  Fiducial.TYPE_USER1: "user_1",
                  Fiducial.TYPE_USER2: "user_2"}

SIMPLE_ACTUATORS = set((SEARCH_PROP, FOLLOW_PROP, DEFUSE_PROP, EXPLORE_PROP))
VALIDATE_TARGET_ACTUATORS = set((DEFUSE_PROP))

# Semantics constants
KNOWN_ACTIONS = set((SEARCH_ACTION, GO_ACTION, GET_ACTION, FOLLOW_ACTION, SEE_ACTION, 
                     TELL_ACTION))
THEME = "Theme"
LOCATION = "Location"
PATIENT = "Patient"
SOURCE = "Source"
UNDERSPECIFIED = "*"

# Response constants
OKAY = "Understood. I'm carrying out your orders now."
FAILURE = "Sorry, I can't make a plan from those orders. Here's why: "
DUNNO = "Sorry, I don't know how to %s."
DUNNO_OBJECT = "Sorry, I don't know how anything about %s."
GOTIT = "Got it. I'll %s."
MISUNDERSTAND = "Sorry, I didn't understand that at all."

# Location to store temp files. To make these true temp files, you need to set this to
# tempfile.mkdtemp(prefix='ltltmp') and then make the dir/file path join properly
OUT_FILE_PREFIX = 'ltlgen'


class StateManager:
    """Provide semantics/LTL interface and automaton management."""

    def __init__(self, nlmaster=None):
        # The automaton and publisher are filled in later so the state manager
        # can start up early
        self.aut = None
        self.pub = None
        self.basedir = "."
        
        self.rooms = None
        self.sensors = None
        self.actuators = None
        self.custom_props = None
        self.sensor_handler = None
        self.motion_handler = None
        self.location = None
        self.room_adjacency = None
        self.room_map = None
        self.last_orders = None
        self.last_other_commands = []
        self.waiting = False
        
        # Thread safety
        self.resynthesizing = False
        self.motion_lock = Lock()
        
        # Connection to a knowledge object, used to check on tells. Will be set by
        # others if neded
        self.world_knowledge = None
        # Similarly, the nlmaster
        self.nlmaster = nlmaster
        
    def load_test_automaton(self, aut_path, movement_test_mode):
        """Load the testing automaton with preset values."""
        rooms = ["r_" + room for room in ("hall", "classroom", "outer_lab")]
        sensors = ["bomb", "user1", "hostage", "search_done"]
        actuators = ["get_defuser", "search"]
        custom_props = ["work", "done", "s_classroom", "s_outer_lab", "have_defuser"]
        self.load_automaton(aut_path, rooms, sensors, actuators, custom_props, None, False,
                            movement_test_mode)

    def load_automaton(self, aut_path, rooms, sensors, actuators, custom_props, room_map,
                       ltl_generator, is_follow, test_mode=False):
        """Load an automaton from a given path."""
        print "Loading automaton from", aut_path
        self.rooms = rooms
        self.sensors = sensors
        self.actuators = actuators
        self.custom_props = custom_props
        self.room_map = room_map
        
        # Initialize all sensors to zero if needed and load the automaton
        if not self.sensor_handler:
            self.sensor_handler = SensorHandler(dict([(sensor, 0) for sensor in sensors]))        
        if not self.motion_handler:
            self.motion_handler = MotionHandler(self, test_mode)
        self.aut = Automaton(self.rooms, self.sensor_handler, ActuatorHandler(self), 
                             self.motion_handler)
        self.aut.loadFile(aut_path, sensors, actuators, custom_props, ltl_generator, is_follow)

    def stop(self):
        """Stop the currently executing automaton and unload it."""
        self.aut.stop()
        self.aut = None

    def set_publisher(self, pub):
        """Set the ROS publisher."""
        self.pub = pub

    def set_basedir(self, path):
        """Set the base directory to call LTLGeneration scripts from."""
        self.basedir = path
    
    def process_orders(self, orders, init_props=None, test_mode=False, world_map=None, stop=True):
        """Process the given orders."""
        # Stop if we need to
        if stop and self.aut:
            self.stop()

        # Put in empty init_props
        if not init_props:
            init_props = ()
                    
        print "Processing orders:", str(orders)
        
        # Update rooms first or use test data
        print "Getting updated world map..."
        self.rooms, self.room_adjacency = get_adjacent_rooms() if not world_map else world_map

        # Check for a bad room
        if self.location not in self.rooms:
            result = "can't process orders from the invalid room %s." % str(self.location)
            return (False, FAILURE + result)

        # Group order by command and validate
        follow_commands = [arguments[THEME] for command, arguments in orders 
                           if command == FOLLOW_ACTION and THEME in arguments]
        if len(follow_commands) > 1:
            result = "can't follow more than one target."
            return (False, FAILURE + result)
        
        follow_command = follow_commands[0] if follow_commands else None
        print "Follow:", follow_command
        
        # Deduplicate these
        search_commands = list(set([arguments[LOCATION] for command, arguments in orders 
                                    if command == SEARCH_ACTION and LOCATION in arguments]))
        print "Search:", search_commands
        
        go_commands = [arguments[LOCATION] for command, arguments in orders 
                       if command == GO_ACTION and LOCATION in arguments]
        if len(go_commands) > 1:
            result = "can only satisfy one go command at a time."
            return (False, FAILURE + result)
        
        go_command = go_commands[0] if go_commands else None
        print "Go:", go_command
        
        get_commands = list(set([(arguments[SOURCE], arguments[THEME]) for command, arguments in orders 
                        if command == GET_ACTION and SOURCE in arguments and THEME in arguments]))
        print "Get:", get_commands
        
        # Check for underspecified gets
        for command in get_commands:
            if UNDERSPECIFIED in command:
                result = "can't carry out underspecified the get command %s." % str(command)
                return (False, FAILURE + result)
        
        # Get other commands, but ignore them if we can't make sense of them
        other_commands = []
        for command, arguments in orders:
            if command not in KNOWN_ACTIONS:
                try:
                    other_commands.append((command, arguments[THEME]))
                except KeyError:
                    print "Warning: Ignoring command %s, don't know how to do it." % \
                        repr(command)

        other_commands = list(set(other_commands))
        print "Other:", other_commands
        
        # Carry over standing orders
        if self.last_other_commands:
            print "Carrying over standing orders:", self.last_other_commands
            other_commands = list(set(other_commands + self.last_other_commands))
        
        # Note whether we're exploring
        exploring = search_commands == ['floor']

        # Check whether there's nothing to be done
        if not (go_command or follow_commands or search_commands or exploring):
            result = "I'd like to carry out your orders, but you haven't told me anything about where to go."
            return (True, result)      

        # Rule out mutually exclusive commands
        if follow_command and (go_command or search_commands):
            result = "can't follow and go/search in the same commands."
            return (False, FAILURE + result)

        # Check for going to places we can't handle yet
        rooms_to_visit = search_commands + [self.location] + ([go_command] if go_command else [])
        bad_rooms = [room for room in rooms_to_visit if room not in self.rooms]
        # Skip check if we're exploring
        if bad_rooms and not exploring:
            result = "can't plan using the rooms %s because they aren't on the map." % str(bad_rooms)
            return (False, FAILURE + result)
        
        # Wait until we stop moving
        print "Waiting for motion lock..."
        with self.motion_lock:
            print "Got motion lock."
        
            # Set the resynth flag so we stop processing environment updates
            self.resynthesizing = True
            
            # Generate and load        
            ltl_gen = LTLGenerator(self.rooms, self.room_adjacency, go_command, get_commands, 
                                   search_commands, follow_command, other_commands)
            # Check for tells before generating
            tells = self.world_knowledge and self.world_knowledge.watch_list
            try:
                (aut_path, self.custom_props, self.sensors, self.actuators, room_map) = \
                    ltl_gen.generate(self.location, init_props, OUT_FILE_PREFIX, self.basedir, tells)
            except UnrealizableError:
                result = "specification is unrealizable."
                print result
                self.resynthesizing = False
                return (False, FAILURE + result)
            
            # Load it up, noting whether it's in follow mode
            follow_mode = bool(follow_command) or exploring
            self.load_automaton(aut_path, self.rooms, self.sensors, self.actuators, self.custom_props, 
                                room_map, ltl_gen, follow_mode, test_mode)
            # Stash away the orders
            self.last_orders = orders
            self.last_other_commands = other_commands
            self.resynthesizing = False
            return (True, OKAY)

    def process_sensor_data(self, msg, world_map=None, test_mode=False):
        """Process updates to the environment and return whether the automaton is done"""
        # If we're resynthesizing, ignore the data
        if self.resynthesizing:
            print "Ignoring environment data during resynthesis."
            return False
                       
        # Processing sensors happens right away so that the current room is always updated
        visible_items = msg.scene.visible
        current_room = None
        activated_sensors = []
        for item in visible_items:
            if item.type == Fiducial.TYPE_REGION:
                current_room = item.id
            elif item.type in SENSOR_MAPPING:
                activated_sensors.append(SENSOR_MAPPING[item.type])
            else:
                # Support things without real types
                activated_sensors.append(item.id)
        
        # Update the room
        if current_room and self.location != current_room:
            print "Setting current room to", current_room
            self.location = current_room
                    
        # Check the map server to see if we're up to date
        map_changed = False
        new_rooms, new_room_adjacency = get_adjacent_rooms() if not world_map else world_map
        # If we're filling in data for the first time, note it
        if not self.room_adjacency or adjacency_changed(self.room_adjacency, new_room_adjacency):
            if not self.room_adjacency:
                print "Received map information for the first time:"
            else:
                print "World map has changed:"
            print new_room_adjacency
            self.rooms = new_rooms
            self.room_adjacency = new_room_adjacency
            map_changed = True          
        
        # If we don't have an automaton, exit at this point
        if not self.aut:
            if not self.waiting or map_changed:
                print "No automaton loaded, so ignoring sensor status."
                self.waiting = True
            return False
        
        if self.aut.isInitialized() and self.aut.isDone():
            # We're already done, so return True
            return True

        # Build dict of environment
        env = dict([(item.lower(), 1) for item in activated_sensors 
                    if item.lower() in self.sensors])

        # Fill in rest with defaults
        for sensor in self.sensors:
            if sensor not in env:
                env[sensor] = 0
        
        self.sensor_handler.updateSensors(env)
        
        # If the map has changed, resynthesize
        if map_changed:
            print "Resynthesizing due to map change..."
            current_props = self.aut.getCurrentOuputs()
            print "Current outputs are:", current_props
            self.process_orders(self.last_orders, current_props, test_mode, world_map, stop=False)
        
        # Initialize the automaton if it needs to be done, and then step
        if not self.aut.isInitialized():
            if current_room:
                # The env is different in this case: keep only the ones that are supposed
                # to be true
                init_sensors = [sensor for sensor in env if env[sensor]]
                # Add on the current props if we resynthesized
                if map_changed:
                    init_sensors += current_props
                    
                # If we're following, lie about the room
                init_room = current_room if not self.aut.is_follow else None
                print "Initializing automaton in room", init_room
                self.aut.chooseInitialState(init_room, init_sensors, map_changed)
            else:
                print ("Warning: Cannot initialize automaton without knowing the location. "
                       "Waiting for another message...")
                self.waiting = False
                return False

        if self.aut.runIteration():
            # If something changed as a result of the iteration, print about it
            print "Transition was made based on environment data:"
            print [item for item in activated_sensors]
            print "Current location is", self.location
            print
        else:
            print "\rWaiting for a state transition, sensors:", \
                [item for item in activated_sensors],

        # Return whether or not the automaton is done executing
        if self.aut.isDone():
            print "Automaton has completed execution. Future environment messages will be ignored."
            print "Mission accomplished!"
            self.notify_done()
            return True
        else:
            self.waiting = False
            return False

    def notify_actuator(self, action, target):
        """Notify the commander that we're actuating."""
        # Skip notification on explore and search
        if action in (EXPLORE_PROP, SEARCH_PROP):
            return

        response = ("I'm now going to %s the %s in the %s." % (action, target, self.location) 
                    if target else  "I'm now going to %s in the %s." % (action, self.location))
        if self.nlmaster:
            self.nlmaster.send_response(response)
        else:
            print response

    def notify_done(self):
        """Notify the commander that we're done."""
        response = "I'm done and I'm in the %s." % self.location
        if self.nlmaster:
            self.nlmaster.send_response(response)
        else:
            print response


def make_response(new_commands):
    """Make a response based on the new commands."""
    # Give up if there were no new commands
    if not new_commands:
        return MISUNDERSTAND

    # Split into good and bad commands, further filtering the good ones
    good_commands = []
    bad_commands = []
    bad_targets = []
    for verb, target in new_commands:
        # Skip the SEE_ACTION entirely
        if verb == SEE_ACTION or (verb in ACTION_ALIASES and ACTION_ALIASES[verb] == SEE_ACTION):
            continue

        # Filter to known verbs, aliasing if needed
        if verb in ACTION_ALIASES or verb in SIMPLE_ACTUATORS or verb in COMPLEX_ACTUATORS:
            # Perform target check for actuators
            if verb in VALIDATE_TARGET_ACTUATORS:
                target_name = target[THEME] if THEME in target else None
                good_target = target_name in SENSOR_MAPPING.values()
            else:
                # Give it a freebie otherwise
                good_target = True

            try:
                verb = ACTION_ALIASES[verb]
            except KeyError:
                pass
            
            if good_target:
                good_commands.append((verb, target))
            else:
                print "Bad target:", target
                if THEME in target:
                    bad_targets.append(target[THEME])
        else:
            bad_commands.append((verb, target))


    # Build up the response
    response = ""
    if good_commands:
        response += GOTIT % _join_commands([_englishify_command(command) for command in good_commands])

    if bad_commands:
        # Pad the initial response if there's something there
        if response:
            response += " "

        response += DUNNO % _join_commands([_englishify_command(command) for command in bad_commands])

    if bad_targets:
        if response:
            response += " "

        response += DUNNO_OBJECT % _join_commands(bad_targets)

    # Return the response if we made it successfully. The expected case
    # where we wouldn't is if there's only a SEE_ACTION command.
    return response if response else MISUNDERSTAND


def _join_commands(commands):
    """Join the commands in a semi-grammatical fashion."""
    # Put in ands and commas as needed
    if len(commands) == 1:
        actions = commands[0]
    elif len(commands) == 2:
        actions = " and ".join(commands)
    else:
        actions = ", ".join(commands[:-1])
        actions += ", and " + commands[-1]

    return actions


def _englishify_command(command):
    """Turn a command into its reasonably grammatical equivalent."""
    verb, target = command

    # Check for known argument structures
    determiner = "the"
    preposition = None
    obj2 = None
    
    # Some specific rules for extracting targets
    if LOCATION in target:
        preposition = "to" if verb != SEARCH_PROP else None
        obj1 = target[LOCATION]
    elif PATIENT in target:
        obj1 = target[PATIENT]
    elif THEME in target:
        obj1 = target[THEME]
        if SOURCE in target:
            # Always gonna be retrieve X from Y
            obj2 = "from " + target[SOURCE]
    else:
        # If we're this lost, print a warning and grab whatever key we can
        print "Warning: Couldn't understand this target when building a response:", target
        try:
            obj1 = target.values()[0]
        except IndexError:
            obj1 = "unknown"

    # Convert the action if possible
    try:
        verb = ACTION_ALIASES[verb]
    except KeyError:
        if verb in SIMPLE_ACTUATORS or verb in COMPLEX_ACTUATORS:
            determiner = "all"
            # Pluralize the target the hackish way
            obj1 += "s"
        else:
            # Switch to indefinite on unknowns
            determiner = "a"

    # Build the response
    response = " ".join((verb, preposition, determiner, obj1)) if preposition else \
        " ".join((verb, determiner, obj1))

    if obj2:
        response += " " + obj2

    return response


class SensorHandler:
    """Sensor handler that reports the environment status."""

    def __init__(self, sensors):
        # Dictionary of the sensors (string) and their readings (int) 
        self.sensors = None
        self.updateSensors(sensors)

    def updateSensors(self, sensors):
        """Update sensors to the given values, handling any required conversions."""
        self.sensors = sensors
        
    def getSensorValue(self, sensor):
        """Return a random value for any sensor after the first time."""
        return self.sensors[sensor]

        
class ActuatorHandler:
    """Send actuations as ROS messages."""
    
    def __init__(self, state_mgr):
        # LTL command publisher
        self.state_mgr = state_mgr
    
    def setActuators(self, actuator_values):
        """Send the actuator actions."""
        parsed_values = []
        for actuator, value in actuator_values:
            if int(value):
                print "ActuatorHandler: Setting actuator", actuator, "to", value
            else:
                print "ActuatorHandler: Taking no action on zero setting of", actuator
                continue        
        
            # Parse the action
            action_match = ACTION_RE.match(actuator)
            action = action_match.group('action')

            # Verify that it's either a simple or complex action
            if action_match and action in COMPLEX_ACTUATORS:
                actuator = COMPLEX_ACTUATORS[action]
                value = action_match.group('target')
            elif actuator in SIMPLE_ACTUATORS:
                # No target for a simple action
                value = None
            else:
                raise ValueError("Don't know how to %s. " % actuator +
                                 "Maybe it should be a custom proposition and not an actuator.")

            # Notify about the actuator and store the result for the command
            self.state_mgr.notify_actuator(action, value)
            parsed_values.append((actuator, value))

        send_ltl_commands(parsed_values, self.state_mgr.pub)

    def stopActuators(self):
        """Stop all actuators."""
        print "Stopping automaton by request."
        send_ltl_commands((), self.state_mgr.pub)
    
class MotionHandler:
    """Send drive commands as ROS messages."""

    def __init__(self, state_mgr, test_mode=False):
        self.state_mgr = state_mgr
        self.next_region = None
        self.test_mode = test_mode
        self.have_lock = False

    def gotoRegion(self, current_region, next_region):
        """Send a drive message to the specified region and return True if we arrive."""
        if current_region == next_region:
            # We don't print anything as this message would come up all the time
            # But do release the lock, as we will be stopping motion if we were moving.
            if self.have_lock:
                self.state_mgr.motion_lock.release()
                self.have_lock = False
            self.next_region = None
            return False
        elif self.next_region == next_region:
            # In test mode, always get there
            if self.test_mode or self.state_mgr.location == next_region:
                # For the sake of test mode, lie about the true location
                if self.test_mode:
                    self.state_mgr.location = next_region
                print "MotionHandler: Finished moving from", current_region, "to", next_region
                self.state_mgr.motion_lock.release()
                self.have_lock = False
                print "Released motion lock."
                return True
            else:
                # Keep doing what we're doing
                return False         
        else:
            if self.have_lock:
                print "Already have lock"
            else:
                print "Acquiring lock..."
            
            if self.have_lock or self.state_mgr.motion_lock.acquire(False):
                self.have_lock = True
                print "Got motion lock."
                print "MotionHandler: Moving from", current_region, "to", next_region
                self.next_region = next_region
                room_name = next_region
                send_ltl_commands(((DRIVE_COMMAND, room_name),), self.state_mgr.pub)
                return False
            else:
                print "MotionHandler: Couldn't get motion lock. Not moving."
                return False
       
