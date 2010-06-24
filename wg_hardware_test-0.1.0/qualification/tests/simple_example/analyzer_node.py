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
import time

import rospy

import matplotlib.pyplot as plt
from StringIO import StringIO

if (len(sys.argv) <= 1):
  rospy.logerr('Must specify one of pass/fail/human')
  sys.exit(0)

rospy.init_node("test_analyzer")

test_service = rospy.ServiceProxy('self_test', Empty)
result_service = rospy.ServiceProxy('test_result', TestResult)

rospy.wait_for_service('self_test')
test_service()

r = TestResultRequest()
r.plots = []
r.params = []
r.params.append(TestParam('P Gain', '5.0'))
r.params.append(TestParam('I Gain', '1.0'))
r.params.append(TestParam('D Gain', '0.0'))
r.params.append(TestParam('I Clamp', '0.0'))

r.values = []
r.values.append(TestValue('Effort', '4.0', '2.0', '5.0'))
r.values.append(TestValue('Low Range', '-2.0', '', '-1.5'))
r.values.append(TestValue('High Range', '2.0', '1.5', ''))


if (sys.argv[1] == "pass"):
  r.html_result = "<p>Test succeeded.</p>"
  r.text_summary = "Test passed."
  r.result = TestResultRequest.RESULT_PASS
  r.values.append(TestValue('Velocity', '2.0', '1.5', '2.5'))
elif (sys.argv[1] == "fail"):
  r.html_result = "<p>Test failed.</p>"
  r.result = TestResultRequest.RESULT_FAIL
  r.text_summary = "Test Failed."
  r.values.append(TestValue('Velocity', '1.0', '1.5', '2.5'))
else:
  r.text_summary = "Human input required."
  r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
  r.values.append(TestValue('Velocity', '1.5', '1.5', '2.5'))  

  plt.plot([1,2,3,4],[16, 9, 4, 1], 'ro')
  plt.xlabel("Pirates")
  plt.ylabel("Ninjas")
  stream = StringIO()
  plt.savefig(stream, format="png")
  image = stream.getvalue()
  
  r.html_result = "<p>Does the correlation between pirates and ninjas make sense?</p>\n<br><img src=\"IMG_PATH/pirates_and_ninjas.png\", width = 640, height = 480 />"
  
  p = Plot()
  r.plots.append(p)
  p.image = image
  p.image_format = "png"
  p.title = "pirates_and_ninjas"
    
# block until the test_result service is available
rospy.wait_for_service('test_result')
result_service.call(r)
rospy.spin()

