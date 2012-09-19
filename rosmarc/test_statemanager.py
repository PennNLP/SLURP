#!/usr/bin/python

import sys
import os

import roslib
roslib.load_manifest('NL')

# Hack path before imports
MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(MODULE_DIR, '..', '..'))

from statemanager import StateManager
from subtle_msgs.msg import Fiducial

INIT_ROOM = "hall"
TEST_WORLD_MAP = (["hall", "library", "classroom"], 
                  [("hall", "hall"), ("hall", "library"), ("hall", "classroom"),
                   ("library", "library"), ("classroom", "classroom")])
LTLGEN_BASE_DIR = os.path.join(MODULE_DIR, '..', '..', 'LTLMoP', 'src', 'etc', 'jtlv')

class FakePublisher:
    """A fake LTL publisher."""
    
    def __init__(self):
        pass
    
    @staticmethod
    def publish(ltl_packet):
        """Do nothing."""
        pass

class FakeSensor:
    """A fake sensor."""
    
    def __init__(self, description, s_type):
        self.id = description
        self.type = s_type

    def __str__(self):
        return self.id

class FakeScene:
    """A fake scene."""
    
    def __init__(self, sensors):
        self.visible = sensors


class FakeEnvMsg:
    """A fake message from the robot about the environment."""
    
    def __init__(self, scene):
        self.scene = scene
        
    def __str__(self):    
        return "FakeEnvMsg: " + str(self.scene.visible)

def make_fake_msg(sensors, types):
    """Make a fake environment message from sensors."""
    return FakeEnvMsg(FakeScene([FakeSensor(sensor, s_type) 
                                 for sensor, s_type in zip(sensors, types)]))


def test_smgr(smgr, room=INIT_ROOM, stop=False):
    """Run the state manager through its automaton."""
    sensors = [room]
    types = [Fiducial.TYPE_REGION]
    user = False
    while not smgr.process_sensor_data(make_fake_msg(sensors, types), TEST_WORLD_MAP, True):
        if smgr.aut.current_outputs["search"]:
            sensors = ["search_done"]
            types = [Fiducial.TYPE_BLOB]
            user = False
            if stop:
                return
        elif user:
            sensors = ["user"]
            types = [Fiducial.TYPE_BLOB]
        else:
            sensors = []
            types = []


def test_fixed(aut_path):
    """Test the state manager on a fixed automaton."""
    smgr = StateManager()
    smgr.set_publisher(FakePublisher)
    smgr.load_test_automaton(aut_path, True)
    test_smgr(smgr)


def test_dynamic():
    """Test the state manager's LTL generation."""
    global TEST_WORLD_MAP    
    orders = eval("""[('search', {'Theme': 'user_1', 'Location': 'library'}), ('search', {'Theme': 'user_1', 'Location': 'classroom'}), ('retrieve', {'Source': 'user_1', 'Theme': 'defuser'}), ('go', {'Theme': '*', 'Location': 'hall'})]""")

    smgr = StateManager()
    smgr.set_basedir(LTLGEN_BASE_DIR)
    smgr.set_publisher(FakePublisher)
    smgr.location = INIT_ROOM
    
    # Add some rooms and synthesize
    new_rooms = ["room" + str(num) for num in range(15)]
    new_world_map = (TEST_WORLD_MAP[0] + new_rooms, TEST_WORLD_MAP[1] + 
                     [("hall", room) for room in new_rooms])
    # Sneakily replace this under the tester's nose
    TEST_WORLD_MAP = new_world_map
    smgr.process_orders(orders, None, True, new_world_map)
    test_smgr(smgr)
    
    
def test_resynth():
    """Test the state manager's LTL resynthesis."""
    global TEST_WORLD_MAP
    orders = eval("""[('search', {'Theme': 'user_1', 'Location': 'library'}), ('search', {'Theme': 'user_1', 'Location': 'classroom'}), ('retrieve', {'Source': 'user_1', 'Theme': 'defuser'}), ('go', {'Theme': '*', 'Location': 'hall'})]""")

    smgr = StateManager()
    smgr.set_basedir(LTLGEN_BASE_DIR)
    smgr.set_publisher(FakePublisher)
    smgr.location = INIT_ROOM
    smgr.process_orders(orders, None, True, TEST_WORLD_MAP)
    test_smgr(smgr, stop=True)
    
    # Add some rooms and synthesize
    print "***Stopping to resynthesize***"
    new_rooms = ["room" + str(num) for num in range(15)]
    new_world_map = (TEST_WORLD_MAP[0] + new_rooms, TEST_WORLD_MAP[1] + 
                     [("hall", room) for room in new_rooms])
    # Sneakily replace this under the tester's nose
    TEST_WORLD_MAP = new_world_map
    test_smgr(smgr, room=smgr.location)


if __name__ == "__main__":
    try:
        test_fixed(sys.argv[1])
    except IndexError:
        test_resynth()
