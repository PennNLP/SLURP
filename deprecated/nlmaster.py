#!/usr/bin/env python
"""NL Master Server, which runs all other components needed."""
import os
import sys
import time
#import rpdb2 
#rpdb2.start_embedded_debugger('nlmaster')

# Stash the path before any more imports are performed
MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
LTLGEN_BASE_DIR = os.path.join(MODULE_DIR, '..', 'LTLMoP', 'src', 'etc', 'jtlv')

# Don't move the ros imports or very bad things might happen
import roslib
roslib.load_manifest('NL')
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from tf import TransformListener
from tf import ExtrapolationException

from semantics.knowledge import Knowledge, rename_entity
from jr.statemanager import StateManager, make_response
from jr.nlproxy import add_subscriber, add_ltl_publisher
from commproxy import CallbackSocket
from pennpipeline import parse_text, init_pipes, close_pipes
from jr.mapcontrol import iPadConnection, MapProxy, RobotPositionProxy
from jr.directions import DirectionProxy

LTL_ENVIRONMENT_TOPIC = "LTLInfo"
ODOM_TOPIC = "odom"
LOCATION_TOPIC = "location"

# Global to allow callbacks to access it
WORLD_KNOWLEDGE = None

class NLMaster:
    """Natural language master server."""
    
    def __init__(self, text_port, map_port):
        self.text_port = text_port
        self.map_port = map_port
        self.ipad_conn = None
        self.comm_proxy = None
        self.map_proxy = None
        self.state_mgr = None
        
    def run(self, aut_path=None):
        """Intialize all NL components."""
        # pylint: disable=W0603
        global WORLD_KNOWLEDGE
    
        # Start the NL pipeline
        if not aut_path:
            print "Starting NL pipeline..."
            init_pipes()
        else:
            print "Skipping loading nlpipeline because an automaton was loaded"
     
        # Start the ROS node
        print "Initializing ROS node..."
        rospy.init_node('nlproxy')
    
        # Set up the state mgr and its callbacks
        print "Starting state manager..."
        self.state_mgr = StateManager(self)
        self.state_mgr.set_basedir(LTLGEN_BASE_DIR)
    
        # Load the automaton if needed
        if aut_path:
            self.state_mgr.load_test_automaton(aut_path, False)
    
        # Create world knowledge
        print "Starting knowledge..."
        WORLD_KNOWLEDGE = Knowledge(self)
        self.state_mgr.world_knowledge = WORLD_KNOWLEDGE
        
        # Wait a little for ROS to avoid timing startup issues.
        print "Waiting for ROS node to settle..."
        time.sleep(1)
        
        # Set up ROS listening
        print "Subscribing to notifications..."
        # Set up state manager's sending and listening
        pub = add_ltl_publisher()
        self.state_mgr.set_publisher(pub)
        add_subscriber(LTL_ENVIRONMENT_TOPIC, self.state_mgr.process_sensor_data)
        add_subscriber(LTL_ENVIRONMENT_TOPIC, WORLD_KNOWLEDGE.process_sensor_data)

        # Create the input comm_proxy and iPad connections
        print "Starting comm_proxy..."
        self.comm_proxy = CallbackSocket(self.text_port)
        self.comm_proxy.register_callback(self.process_text)
        self.ipad_conn = iPadConnection(self.map_port)
        self.ipad_conn.register_rename_callback(rename_entity)
        self.ipad_conn.register_text_callback(self.process_text)
        # TODO Add highlight callback
        self.map_proxy = MapProxy(self.ipad_conn)
        add_subscriber(LTL_ENVIRONMENT_TOPIC, self.ipad_conn.add_icons)        
        WORLD_KNOWLEDGE.map_proxy = self.map_proxy

        # Set up odometry forwarding to the ipad
        tf = TransformListener()
        while not tf.frameExists("/map"):
            rospy.logwarn("Not ready for transforms yet")
            rospy.sleep(1.0)
        position_proxy = RobotPositionProxy(self.ipad_conn,tf)
        rospy.Subscriber(ODOM_TOPIC, Odometry, position_proxy.forward_position)

        # Set up getting directions
        direction_proxy = DirectionProxy()
        rospy.Subscriber(LOCATION_TOPIC, String, direction_proxy.set_location)
        WORLD_KNOWLEDGE.direction_proxy = direction_proxy
        
        print "NLMaster startup complete!"
        print "*" * 80
    
        # Finally, wait for input before quitting
        try:
            while True:
                text = raw_input("").strip()
                if text == "q":
                    break
        except (KeyboardInterrupt, EOFError):
            pass
        except:
            raise
        finally:
            # Only shutdown the pipeline if we actually were taking language input
            if not aut_path:
                print "Shutting down NL pipeline..."
                close_pipes()
            self.comm_proxy.shutdown()
            self.ipad_conn.shutdown()
    
        sys.exit()

    def process_text(self, text):
        """Parse text and pass it to semantics."""
        if text:
            # Clean unicode first if needed
            if isinstance(text, unicode):
                text = text.encode('ascii', 'ignore')
            
            print "Parsing:", repr(text)
            parse = parse_text(text)
            result = WORLD_KNOWLEDGE.process_parse_tree(parse, text)
            answer = result[0]
            new_commands = result[3]
            # Use the answer if there was one and it was a string
            response = answer if answer and isinstance(answer, str) else ""
            
            # Also, echo back any new commands
            command_echo = make_response(new_commands)
                
            # Combine the responses as needed.
            if not response:
                response = command_echo
            elif new_commands:
                response += " " + command_echo

            if response:
                self.send_response(response)
            
            print "Completed:", repr(text)


    def send_response(self, text):
        """Send text to all listeners."""
        print "Sending text to listeners:", repr(text)                
        self.ipad_conn.sendtext(text)
        self.comm_proxy.send(text)

if __name__ == '__main__':
    try:
        TEXT_PORT = int(sys.argv[1])
        MAP_CONTROL_PORT = int(sys.argv[2])
    except (IndexError, ValueError):
        print >> sys.stderr, "Usage: nlmaster textport mapport [automaton]"
        sys.exit(2)

    master = NLMaster(TEXT_PORT, MAP_CONTROL_PORT)
    try:
        master.run(sys.argv[3])
    except IndexError:
        master.run()
