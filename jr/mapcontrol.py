#!/usr/bin/env python
"""Supports communication with the iPad's map interfaces."""

import json
from threading import Thread, Lock
from time import sleep

from commproxy import CallbackSocket
from worldmap import get_topo_map, get_occupancy_grid
from subtle_msgs.msg import Fiducial
from tf import TransformListener
from tf import ExtrapolationException
from geometry_msgs.msg import *
import rospy

IPAD_HEIGHT = 669
IPAD_WIDTH = 900
MAP_HEIGHT = None
MAP_WIDTH = None
MAP_ORIGIN = None
MAP_RESOLUTION = None
HAVE_MAP = False

ICON_MAP = {Fiducial.TYPE_BOMB: "icon-bomb",
            Fiducial.TYPE_HOSTAGE: "icon-hostage",
            Fiducial.TYPE_BADGUY: "icon-badguy",
            Fiducial.TYPE_USER1: "icon-user1",
            Fiducial.TYPE_USER2: "icon-user2"}


def SerializeAction(event, argument):
    """Serialize an action message."""
    validActions = ['highlight', 'rename', 'showimage', 'hideimage', 'text']
    #make sure event is one of the above
    validActions.index(event)
    # Create the JSON
    tmp = dict()
    tmp['type'] = "action"
    tmp['data'] = dict()
    tmp['data']['event'] = event
    tmp['data']['argument'] = argument
    return json.dumps(tmp)


def SerializeAddObj(obj_name, obj_type, location):
    '''Serialize Add Obj Request
    @param obj_name String name of map object
    @param obj_type String either "region" or "icon"
    @param location [[x1,y1], [x2,y2] ...] - should only be one "point" for icon'''
    validObjTypes = ['region', 'icon-bomb', 'icon-hostage', 'icon-badguy',
                     'icon-user1', 'icon-user2', 'icon-robot']
    validObjTypes.index(obj_type)
    new_obj = dict()
    new_obj['type'] = "mapupdate"
    new_obj['data'] = dict()
    new_obj['data']['event'] = "addobj"
    new_obj['data']['name'] = obj_name
    new_obj['data']['params']  =  dict()
    new_obj['data']['params']['type'] = obj_type
    new_obj['data']['params']['location'] = location
    return json.dumps(new_obj)


def SerializeRemObj(obj_name, obj_type):
    ''' Serialize Rem Obj Request 
    @param obj_name String name of map object'''
    new_obj = dict()
    new_obj['type'] = "mapupdate"
    new_obj['data'] = dict()
    new_obj['data']['event'] = "remobj"
    new_obj['data']['name'] = obj_name
    new_obj['data']['params'] = dict()
    new_obj['data']['params']['type'] = obj_type
    return json.dumps(new_obj)


class iPadConnection(CallbackSocket):
    """Provides a connection to the iPad app over JSON."""
    name = "iPadConnection"
    
    def __init__(self, port):
        CallbackSocket.__init__(self, port)
        self._rename_callback = None
        self._highlight_callback = None
        self._text_callback = None
        self.register_callback(self.listener)
        self.sent_icons = set()
        self.send_lock = Lock()

    def highlightObject(self, objName):
        """ send highlight command to ipad """
        # @param objName String the name of the object
        self.send(SerializeAction("highlight", objName))

    def showimage(self, objName):
        """ tell ipad to display a picture of object """
        # @param objName String the name of the object
        self.send(SerializeAction("showimage", objName))

    def hideimage(self, objName):
        """ tell ipad to hide a picture of object """
        # @param objName String the name of the object
        self.send(SerializeAction("hideimage", objName))

    def sendtext(self, message):
        """ tell ipad to hide a picture of object """
        # @param objName String the name of the object
        self.send(SerializeAction("text", message))

    def addicon(self, objName, icontype, x, y):
        """ Add an icon to the map """
        # @param objName String name of object
        # @param icontype icon type - bomb, user1, etc...
        # @param x x coordinate
        # @param y y coordinate
        self.send(SerializeAddObj(objName, icontype, [[x, y]]))

    def remicon(self, objname, objtype):
        """ remove an icon from the map """
        self.send(SerializeRemObj(objname, objtype))

    def addregion(self, name, pointslist):
        """ adds a region to the map """
        #@param name String the name of the object
        #@param pointslist [ [x,y], [x,y], [x,y], ...]
        self.send(SerializeAddObj(name, "region", pointslist))
    
    def remregion(self, name):
        """Send a message to remove a region."""
        self.send(SerializeRemObj(name, "region"))

    def register_highlight_callback(self, function):
        """Register a function as a callback for highlight messages."""
        self._highlight_callback = function

    def register_rename_callback(self, function):
        """Register a function as a callback for rename messages."""  
        self._rename_callback = function

    def register_text_callback(self, function):
        """Register a function as a callback for text messages."""
        self._text_callback = function

    def listener(self, message):
        """Dispatch callbacks to the appropriate handlers."""
        try:
            data = json.loads(message)
            if data['type'] == "action":
                if data['data']['event'] == "highlight":
                    self._highlight_callback(data)
                elif data['data']['event'] == "text":
                    self._text_callback(data['data']['argument'])
                elif data['data']['event'] == "rename":
                    self._rename_callback(*data['data']['argument'].split('->'))
                elif data['data']['event'] == "point":
                    print "Received point:", str(data['data']['argument'])
                else:
                    print "Unrecognized action received:", data
        except ValueError:
            print "iPadConnection: Couldn't understand message", repr(message)

    def add_icons(self, msg):
        """Send environment updates from an environment message to the iPad."""
        # Require that we be connected and have a map        
        if not (self.is_connected() and HAVE_MAP):
            return

        # Send each item that appeared to the iPad
        for item in msg.scene.appeared:
            try:
                icon = ICON_MAP[item.type]
                name = item.id
                location_x, location_y = item.pose.position.x, item.pose.position.y
                converted_x, converted_y = convert_points(location_x, location_y, MAP_RESOLUTION, 
                    MAP_WIDTH, MAP_HEIGHT, MAP_ORIGIN[0], MAP_ORIGIN[1])
                print "Sending icon to iPad:"
                print item
                self.addicon(name, icon, converted_x, converted_y)
                self.showimage(name)
            except KeyError:
                # Ignore other types
                pass
    

class MapProxy:
    """A daemon to poll the map server for updates and send them to the iPad."""
    name = "MapProxy"
    
    def __init__(self, ipad):
        self.ipad = ipad
        self._last_addr = None
     
        # Start the pumping thread
        callback_thread = Thread(target=self.poll)
        callback_thread.daemon = True
        callback_thread.start()
        
    def poll(self):
        """Get the latest map and forward info to the iPad if things have changed."""
        print "Map Proxy poller waiting for first map..."
        last_regions = []
        
        while True:            
            # Check for a valid map. We can get None if the service is shutting down
            new_map = get_topo_map()
            if not new_map:
                print "%s: Topo map server appears to be down. Giving up." % self.name
                break
            
            new_regions = new_map.regions
            if new_regions != last_regions or self._last_addr != self.ipad.client_addr:
                grid = get_occupancy_grid()
                if not grid:
                    print "%s: Occupancy grid server appears to be down. Giving up." % self.name
                    break

                resolution, width, height, origin_x, origin_y = (grid.resolution, grid.width, 
                    grid.height, grid.origin.position.x, grid.origin.position.y)

                # Stash these for later so everyone can use them
                # pylint: disable-msg=W0603
                global MAP_HEIGHT, MAP_WIDTH, MAP_ORIGIN, MAP_RESOLUTION, HAVE_MAP
                MAP_HEIGHT = height
                MAP_WIDTH = width
                MAP_ORIGIN = (origin_x, origin_y)
                MAP_RESOLUTION = resolution
                HAVE_MAP = True
                    
                # Check for the iPad connection
                if not self.ipad.is_connected():
                    last_regions = []
                    try:
                        sleep(1)
                    except: # pylint: disable=W0702
                        # Ignore exceptions during sleep that can occur during shutdown
                        pass
                    continue

                print "%s: Sending new map to iPad" % self.name
                print "Resolution:", resolution
                print "Width, Height:", width, height
                print "Origin x, y:", origin_x, origin_y

                # Remove any regions from the old_map
                for region in last_regions:
                    self.ipad.remregion(region.name)
                
                # Now add the new ones
                for region in new_regions:
                    print "Sending region", region.name, "to iPad."
                    converted_points = \
                        [convert_points(points.x, points.y, resolution, width, height, origin_x, origin_y) 
                         for points in region.boundary.points]
                    self.ipad.addregion(region.name, converted_points)
                    
                # Store map
                last_regions = new_regions
           
            # Stash the address
            self._last_addr = self.ipad.client_addr
         
            # Wait for a bit
            try:
                sleep(1)
            except: # pylint: disable=W0702
                # Ignore any exception during sleep, generally this during shutdown 
                pass


class RobotPositionProxy:
    """Connects the iPad to odometry information."""
    
    def __init__(self, ipad,tf):
        self.ipad = ipad
        self.last_x = None
        self.last_y = None
        self.tf = tf
        

    def forward_position(self, msg):
        """Forward an odometry message to the ipad."""
        # Require that we be connected and have a map
        if not (self.ipad.is_connected() and HAVE_MAP):
            return


        # tf stuff
        pose = PoseStamped()
        pose.pose = msg.pose.pose
        pose.header = msg.header
        self.tf.waitForTransform("/map",pose.header.frame_id, rospy.Time().now(), rospy.Duration(0.5))
        realpose = self.tf.transformPose("/map",pose)
        # Parse the message
        #x_pos = msg.pose.pose.position.x
        #y_pos = msg.pose.pose.position.y   
        x_pos = realpose.pose.position.x
        y_pos = realpose.pose.position.y

        # Transform the coordinates
        x_converted, y_converted = convert_points(x_pos, y_pos, MAP_RESOLUTION, MAP_WIDTH,
                                                  MAP_HEIGHT, MAP_ORIGIN[0], MAP_ORIGIN[1])

        # Send if anything changed
        if (self.last_x, self.last_y) != (x_converted, y_converted):
            self.last_x, self.last_y = x_converted, y_converted
            self.ipad.addicon("junior", "icon-robot", x_converted, y_converted)
            


def convert_points(x, y, resolution, width, height, origin_x, origin_y):
    """Convert points from the ROS space to the iPad space."""
    new_x = (x - origin_x) / resolution #* (IPAD_WIDTH / float(width))
    new_y = height - ((y - origin_y) / resolution) #* (IPAD_HEIGHT/ float(height)))
    return (new_x, new_y)

# def convert_points(x, y, resolution, width, height, origin_x, origin_y):
#     """Convert points from the ROS space to the iPad space."""
#     new_x = (x - origin_x) / resolution * (IPAD_WIDTH / float(width))
#     new_y = IPAD_HEIGHT - (((y - origin_y) / resolution) * (IPAD_HEIGHT/ float(height)))
#     return (new_x, new_y)
