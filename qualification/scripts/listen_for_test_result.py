#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008-2010, Willow Garage, Inc.
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

##\author Blaise Gassend
##\brief Checks that the imager model and color filter on a wge100 camera
##match its serial number.

PKG = 'qualification'
import roslib
roslib.load_manifest(PKG)
import rospy
from pr2_self_test_msgs.srv import *
import time
import re

def result_cb(req):
    if req.result == TestResultRequest.RESULT_PASS:
        out = rospy.loginfo
        out("Test passed!")
    elif req.result == TestResultRequest.RESULT_FAIL:
        out = rospy.logerr
        out("Test failed!")
    elif req.result == TestResultRequest.RESULT_HUMAN_REQUIRED:
        out = rospy.logwarn
        out("Test needs human intervention!")
    out("Summary: %s"%req.text_summary)
    out("")
    txt = "Detailed results:\n"+req.html_result
    txt = txt.replace("<p>", "")
    txt = txt.replace("</p>", "\n")
    txt = txt.replace("<br>", "\n")
    out(txt)
    global done
    done = True
    return TestResultResponse()

done = False
def main():
    rospy.init_node('listen_for_test_result')
    rospy.Service('test_result', TestResult, result_cb)
    while not done and not rospy.is_shutdown():
        time.sleep(0.1)

if __name__ == "__main__":
    main()
