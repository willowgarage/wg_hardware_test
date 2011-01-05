#! /usr/bin/env python
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\author Kevin Watts
##\brief Raises torso for PR2 burn in test

from __future__ import with_statement

PKG = 'qualification'

import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib
import threading
import sys

from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from pr2_self_test_msgs.srv import TestResult, TestResultRequest
from sensor_msgs.msg import JointState

HEIGHT = 0.28 # in meters
TOLERANCE = 0.10
TORSO_NAME = 'torso_lift_joint'

class LastMessage():
    def __init__(self, topic, msg_type):
        self.msg = None
        rospy.Subscriber(topic, msg_type, self.callback)

    def last(self):
        return self.msg

    def callback(self, msg):
        self.msg = msg

def check_torso_up(msg):
    for i, name in enumerate(msg.name):
        if name == TORSO_NAME:
            return abs(msg.position[i] - HEIGHT) < TOLERANCE
    return False
            
if __name__ == '__main__':
    rospy.init_node('torso_lifter_client')

    _mutex = threading.Lock()

    last_state = LastMessage('joint_states', JointState)

    client = actionlib.SimpleActionClient('torso_controller/position_joint_action',
                                          SingleJointPositionAction)
    rospy.loginfo('Waiting for torso action client')
    client.wait_for_server()

    goal = SingleJointPositionGoal()
    goal.position = HEIGHT
    
    client.send_goal(goal)
    
    # Wait for result, or wait 60 seconds to allow full travel
    client.wait_for_result(rospy.Duration.from_sec(60))

    # Check that torso is mostly up
    ok = False
    msg = None
    with _mutex:
        msg = last_state.last()
        if msg:
            ok = check_torso_up(msg)
            


    r = TestResultRequest()
    if not msg:
        r.text_summary = 'No joint state messages receieved. Unable to tell if torso up'
        r.result = TestResultRequest.RESULT_FAIL
    elif not ok:
        r.text_summary = 'Attempted to drive torso up, unable to do so.'
        r.result = TestResultRequest.RESULT_FAIL
    else:
        r.text_summary = 'Drove torso up, OK'
        r.result = TestResultRequest.RESULT_PASS

    try:
        rospy.wait_for_service('test_result', 5)
    except:
        rospy.logfatal('Unable to contact result service. Exiting')
        sys.exit(-1)

    result_proxy = rospy.ServiceProxy('test_result', TestResult)
    result_proxy.call(r)
    

    rospy.spin()
    

