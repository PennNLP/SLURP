"""Provides tracking the robot and generation of directions to where it is."""

from threading import Lock

import rospy
from rospy import ServiceException, ROSInterruptException
from get_directions.srv import GetDirections, GetDirectionsRequest


# Give directions
class DirectionProxy:
    """Listens for the current location and generates directions from there."""

    def __init__(self):
        self.get_directions = rospy.ServiceProxy('get_directions', GetDirections)
        self.location = None
        self.location_lock = Lock()

    def set_location(self, data):
        """Set the current location."""
        with self.location_lock:
            self.location = data.data

    def give_directions(self, destination):
        """Give directions to a destination from the current location."""
        with self.location_lock:
            if not self.location:
                return "I don't know where I am, so I can't tell you how to get here."

            req = GetDirectionsRequest()
            print "Getting directions:", self.location, destination
            req.endpoints[0] = self.location
            req.endpoints[1] = destination
            # Getting the robots real heading is somewhat of a pain and
            # makes dependencies you probably don't want to deal with.
            # Abe says this really only matters for the first turn, and if you don't
            # care much about that then you can set it to some arbitrary value like i do here.
            req.heading = 0 # in radians
            try:
                directions = self.get_directions(req)
                print "Got directions!"
                return str(directions.directions)
            except (ServiceException, ROSInterruptException) as exc:
                print "Error: Directions call failed: %s" % exc
                return None

