import roslib
roslib.load_manifest('nlp')

from nlp_comms.srv import String, StringResponse
import rospy

def handle_test_service(request):
    return StringResponse("bar")

def string_server():
    rospy.init_node("test_server")
    srv = rospy.Service("test_service", String, handle_test_service)
    print "Service started."
    rospy.spin()

if __name__ == "__main__":
    string_server()
