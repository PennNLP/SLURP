#!/usr/bin/env python
from time import sleep

import rospy
import std_msgs

rospy.init_node('test_publisher')
pub = rospy.Publisher('test_topic', std_msgs.msg.String)
i = 0
while True:
    msg = "hello world %d" % i
    pub.publish(msg)
    sleep(1)
    i += 1
