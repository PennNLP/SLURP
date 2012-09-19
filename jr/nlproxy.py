#!/usr/bin/python
"""Functions for ROS communication."""

import sys

import rospy

from subtle_msgs.msg import LTLAction, LTLPacket

LTL_SEND_TOPIC = "LTLCommand"

def add_subscriber(topic, callback):
    """Add an LTLCommand callback subscriber."""
    rospy.Subscriber(topic, LTLPacket, callback)


def add_ltl_publisher():
    """Add an LTLCommand publisher."""
    return rospy.Publisher(LTL_SEND_TOPIC, LTLPacket)


def send_ltl_commands(action_targets, publisher):
    """Send a command on a publisher."""
    if rospy.is_shutdown(): 
        print >> sys.stderr, "WARNING: Cannot send command when ROS is shutdown."
        return False

    # Fill in the packet's actions
    ltl_packet = LTLPacket()
    for action, target in action_targets:
        ltl_action = LTLAction()
        ltl_action.verb = action
        if target:
            ltl_action.targets.append(target)            
        ltl_packet.actions.append(ltl_action)

    # Send it to the robot
    print "nlproxy: Sending actions", action_targets
    rospy.loginfo("nlproxy: Sending:\n" + str(ltl_packet))
    publisher.publish(ltl_packet)
    return True

