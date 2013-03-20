#!/usr/bin/env python
"""Test basic functionality of the Penn Pipeline over ROS."""

import roslib
roslib.load_manifest('test_nlp')

import sys

import rospy
from nlp.srv import String


PIPELINE_SERVICE = "penn_nlp_pipeline_service"


def parse_input():
    """Request a parse for the given text."""
    print "Waiting for service {}...".format(PIPELINE_SERVICE)
    rospy.wait_for_service(PIPELINE_SERVICE)
    try:
        pipeline = rospy.ServiceProxy(PIPELINE_SERVICE, String)
        while True:
            user_input = raw_input("> ")
            response = pipeline(user_input)
            print response
    except rospy.ServiceException, err:
        print >> sys.stderr, "Service call failed: {}".format(err)
    except (KeyboardInterrupt, EOFError):
        pass


if __name__ == "__main__":
    try:
        parse_input()
    except rospy.ROSInterruptException:
        pass
