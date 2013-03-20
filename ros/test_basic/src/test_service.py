#!/usr/bin/env python
import roslib
roslib.load_manifest('nlp')

import rospy
from nlp.srv import String, StringResponse


def handle_test_service(request):
    return StringResponse("bar")


def string_server():
    rospy.init_node("test_server")
    srv = rospy.Service("test_service", String, handle_test_service)
    print "Service started."
    rospy.spin()


if __name__ == "__main__":
    try:
        string_server()
    except rospy.ROSInterruptException:
        pass
