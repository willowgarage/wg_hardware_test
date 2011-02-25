#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('qualification')

import sys
import rospy
from std_srvs.srv import Empty
from pr2_self_test_msgs.srv import TestResult, TestResultRequest
from pr2_self_test_msgs.msg import Plot, TestParam, TestValue
from turtlebot_node.srv import SetDigitalOutputsRequest, SetDigitalOutputs

rospy.init_node("breaker_test")

result_service = rospy.ServiceProxy('test_result', TestResult)
breaker_service = rospy.ServiceProxy('turtlebot_node/set_digital_outputs', SetDigitalOutputs)

rospy.wait_for_service('turtlebot_node/set_digital_outputs')

my_arg = sys.argv[1]

r = TestResultRequest()
r.text_summary = "Check the kinect power cable."
r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
r.html_result = "<p>Did the light on the kinect cable turn on?</p><br><img src=\"%s/kinect_power.jpg\", width=\"400\", height=\"194\">"%my_arg

rospy.logerr(r.html_result)
try:
    rospy.loginfo("setting digital outs")
    res = breaker_service.call(SetDigitalOutputsRequest(1,0,0))
except rospy.ServiceException, e:
    r.text_summary = "Service call failed: the turtlebot_node is not running startup failed"
    r.result = TestResultRequest.RESULT_FAIL
    r.html_result = "Service call failed: the turtlebot_node is not running startup failed"


# block until the test_result service is available 
rospy.wait_for_service('test_result')
result_service.call(r)
rospy.spin()
