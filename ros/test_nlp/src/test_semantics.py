#!/usr/bin/env python
"""Test basic functionality of the Penn Semantics Service over ROS."""

import roslib
roslib.load_manifest('test_nlp')

import sys
import json

import rospy
from nlp.srv import String


PIPELINE_SERVICE = "penn_nlp_pipeline_service"
SEMANTICS_SERVICE = "penn_nlp_semantics_service"


def parse_input():
    """Request a parse for the given text."""
    print "Waiting for service {}...".format(PIPELINE_SERVICE)
    rospy.wait_for_service(PIPELINE_SERVICE)
    print "Waiting for service {}...".format(SEMANTICS_SERVICE)
    rospy.wait_for_service(SEMANTICS_SERVICE)
    try:
        pipeline = rospy.ServiceProxy(PIPELINE_SERVICE, String)
        semantics = rospy.ServiceProxy(SEMANTICS_SERVICE, String)
        while True:
            user_input = raw_input("> ")
            parse = pipeline(user_input)
            semantics_msg = {'tree': parse.out, 'text': user_input}
            semantics = semantics(json.dumps(semantics_msg))
            print semantics
    except rospy.ServiceException, err:
        print >> sys.stderr, "Service call failed: {}".format(err)
    except (KeyboardInterrupt, EOFError):
        pass


if __name__ == "__main__":
    try:
        parse_input()
    except rospy.ROSInterruptException:
        pass
