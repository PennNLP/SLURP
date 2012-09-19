"""Functions for accessing the map server."""

import roslib
roslib.load_manifest('NL')

import rospy
from rospy import ServiceException, ROSInterruptException
from subtle_msgs.srv import GetTopoMap
from nav_msgs.srv import GetMap

HARD_CODED_ADJACENCY = [("hall", "classroom"), ("hall", "outer_lab")]

def adjacency_changed(adj1, adj2):
    """Return whether two adjacency representations are the same or not."""
    return set(adj1) != set(adj2)


def get_adjacent_rooms():
    """Get a list of rooms that are adjacent to each other."""
    topo_map = get_topo_map()
    # If the service goes down during the request, we'll get None
    if not topo_map:
        return (None, None)
    
    rooms = [region.name for region in topo_map.regions]
    adjacent_rooms = set()

    # Add self-connections, any duplicates will be filtered by the set
    for room in rooms:
        adjacent_rooms.add((room, room))
    
    # Add the room paths between rooms
    for path in topo_map.paths:
        room1, room2 = path.nodes
        
        # Don't add a connection if its inverse is there
        if (room2, room1) not in adjacent_rooms:
            adjacent_rooms.add((room1, room2))

    # Convert because some callers may want them as lists
    return (rooms, list(adjacent_rooms))


def get_topo_map():
    """Get a map from the map server."""
    # Wait for the map server to start
    try:
        rospy.wait_for_service('/getTopoMap')
        map_getter = rospy.ServiceProxy('getTopoMap', GetTopoMap)
        return map_getter().topo_map
    except (ServiceException, ROSInterruptException) as exc:
        print "Error: Map call failed: %s" % exc
        
        
def get_occupancy_grid():
    """Get an occupancy grid from the map server."""
    # Wait for the map server to start
    try:
        try:
            grid_getter = rospy.ServiceProxy('static_map', GetMap)
            return grid_getter().map.info
        except ServiceException:
            #print "No static map service, trying dynamic map."
            pass

        try: 
            grid_getter = rospy.ServiceProxy('dynamic_map', GetMap)
            return grid_getter().map.info
        except ServiceException:
            print "Error: Could not contact static or dynamic map services."
        
    except ROSInterruptException as exc:
        print "Error: Map call failed: %s" % exc
