from time import sleep

import rospy
from nlp.srv import Tag

def tag_tokens(tokens):
    """Tag a string of space-separated tokens."""
    output = (("Test", "NN"),)
    return output

def tagger_server():
    """Run a tagging service."""
    rospy.init_node('tagger_server')
    service = rospy.Service('tag_tokens', nlp.srv.Tag, tag_tokens)
    rospy.spin()


if __name__ == "__main__":
    tagger_server()
