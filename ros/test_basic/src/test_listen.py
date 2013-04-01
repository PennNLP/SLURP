#!/usr/bin/env python
import rospy
import std_msgs

def callback(data):
    rospy.loginfo("I heard %s", data.data)


def listen():
    rospy.init_node('test_listener')
    rospy.Subscriber('test_topic', std_msgs.msg.String, callback)
    rospy.spin()


if __name__ == "__main__":
    listen()
