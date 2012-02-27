import sys
import time
import random

from fsa import Automaton


class DummySensorHandler:
    """Fake sensor handler that randomly returns a value for any reading."""

    def __init__(self):
        self.first = True

    def getSensorValue(self, dummy):
        """Return a random value for any sensor after the first time."""
        if self.first:
            self.first = False
            return 0
        else:
            return random.randint(0, 1)


class DummyActuatorHandler:
    """Fake actuator handler that does nothing."""
    
    def __init__(self):
        pass
    
    def setActuators(self, actuator_values):
        """Print out the actuator actions."""
        for actuator, value in actuator_values:
            print "Setting actuator", actuator, "to", value


class DummyMotionHandler:
    """Fake motion handler that does nothing."""

    def __init__(self):
        self.next_region = None
        self.countdown = 1 # First move is free

    def gotoRegion(self, current_region, next_region):
        if current_region == next_region:
            print "Staying in", current_region
        else:
            print "Moving from", current_region, "to", next_region
            self.countdown -= 1
            if self.countdown < 1:
                print "Arrived!"
                self.countdown = 3
                return True
            else:
                print "Still moving..."
                return False


def test():
    """Test the FSA module."""
    aut_file = sys.argv[1]

    regions = None # The automaton ignores these
    if aut_file == "test_sim.aut":
        sensors = ['hostage']
        actuators = ['wave']
        custom_props = ['done']
    elif aut_file == "scen1.aut":
        sensors = ['user']
        actuators = ['get_defuser']
        custom_props = ['s2', 's3', 'have_defuser', 'done', 'work']
    else:
        raise ValueError("Don't know anything about the automaton", aut_file)

    print "Loading automaton..."
    aut = Automaton(regions, DummySensorHandler(), DummyActuatorHandler(), DummyMotionHandler())
    aut.loadFile(aut_file, sensors, actuators, custom_props)
    print "Initializing automaton..."
    aut.chooseInitialState('r1', ())
    print "Running automaton..."
    while True:
        aut.runIteration()
        print "State:", aut.current_state
        if aut.isDone():
            break
        #time.sleep(1)

    print "Done!"


if __name__ == "__main__":
    test()
