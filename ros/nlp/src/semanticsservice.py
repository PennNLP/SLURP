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
from nlp.srv import String
import rospy

import sys
import os
import json

# Get slurp into the sys.path
MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(MODULE_DIR, '..', '..', '..'))
from semantics.parsing import process_parse_tree


NODE_NAME = "penn_nlp_semantics_server"
SERVICE_NAME = "penn_nlp_semantics_service"

# In-order names to give to return values of the response
SEMANTICS_KEYS = ('user_response', 'new_commands')


def process_text(request):
    """Return a parse response."""
    request_dict = json.loads(request.in_)
    rospy.loginfo("NLP semantics request: %r" % request_dict)
    response = process_parse_tree(request_dict['tree'].encode('ascii', 'replace'), 
                                  request_dict['text'].encode('ascii', 'replace'))
    frames, new_commands, kb_response = response
    user_response = "Got it." if not kb_response else "I understood: " + kb_response

    # TODO: Properly serialize the commands, possibly into a new message format
    command_dicts =  [{'Command': str(command)} for command in new_commands]

    response_dict = dict(zip(SEMANTICS_KEYS, [user_response, command_dicts]))
    response_json = json.dumps(response_dict)
    rospy.loginfo("NLP semantics response: %r" % response_json)
    return response_json


def semantics_server():
    """Run the semantics service."""
    # ROS node setup
    rospy.init_node(NODE_NAME)
    rospy.loginfo("Penn Semantics started on node %s" % NODE_NAME)

    # Start the service
    srv = rospy.Service(SERVICE_NAME, String, process_text)
    rospy.loginfo("Penn Semantics available on service %s" % SERVICE_NAME)
    rospy.spin()


if __name__ == "__main__":
    semantics_server()
