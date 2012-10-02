#!/usr/bin/env python
"""
Hosts requests sent to the NLPipeline as ROS services.

"""

# Copyright (C) 2012 Constantine Lignos
#
# This file is a part of SLURP.
#
# SLURP is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# SLURP is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with SLURP.  If not, see <http://www.gnu.org/licenses/>.

import roslib
roslib.load_manifest('nlp')
from nlp_comms.srv import String, StringResponse
import rospy

import sys
import os
# Get slurp into the sys.path
MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(MODULE_DIR, '..', '..'))

from pennpipeline import parse_text, init_pipes, close_pipes

NODE_NAME = "upenn_nlp_pipeline_server"
SERVICE_NAME = "upenn_nlp_pipeline_service"

class PipelineService():
    """Provides a connection to the pipeline via a ROS service."""
    name = "pipelinehost"
    
    def __init__(self):
        init_pipes()

    def __del__(self):
        # We put this check as it may already be undefined during interpreter shutdown.
        # There's no guarantee this will succeed during shutdown anyway.
        if close_pipes:
            close_pipes()

    def parse_text(self, request):
        """Return a parse response."""
        text = request.in_
        rospy.loginfo("NLP pipeline request: %r" % text)
        response = parse_text(text)
        rospy.loginfo("NLP pipeline response: %r" % response)
        return response


def pipeline_server():
    # ROS node setup
    rospy.init_node(NODE_NAME)
    rospy.loginfo("UPenn NLP Pipeline started on node %s" % NODE_NAME)

    # Pipeline setup
    host = PipelineService()

    # Start the service
    srv = rospy.Service(SERVICE_NAME, String, host.parse_text)
    rospy.loginfo("UPenn NLP Pipeline available on service %s" % SERVICE_NAME)
    rospy.spin()


if __name__ == "__main__":
    pipeline_server()
