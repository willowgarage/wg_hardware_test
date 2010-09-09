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
from std_srvs.srv import *
from pr2_self_test_msgs.srv import *
import std_msgs
import rospy
import subprocess
import os
import os.path
import wx

rospy.init_node("load_firmware_ram", anonymous=True)

r = TestResultRequest()
r.plots = []

try:
    impactdir=rospy.get_param("~impactdir")
except:
    import traceback
    traceback.print_exc()
    print >> sys.stderr, 'impactdir option must indicate impact project directory';
    r.html_result = "<p>Bad arguments to load_firmware.py.</p>"
    r.text_summary = "Error in test."
    r.result = TestResultRequest.RESULT_FAIL
    print "error"
else:
    os.chdir(impactdir);
    p = subprocess.Popen(['./startimpact', '-batch', 'load_firmware_ram.cmd'], stderr=subprocess.PIPE)
    impactout = p.communicate()[1]

    impactout = impactout.replace('\n','<br>')

    if '''INFO:iMPACT - '1': Programing completed successfully.''' in impactout:
	r.text_summary = "Firmware download succeeded."
        r.html_result = "<p>Test passed.</p><p>"+impactout+"</p>" 
        r.result = TestResultRequest.RESULT_PASS
        print "pass"
    else:
        r.text_summary = "Firmware download failed."
        r.result = TestResultRequest.RESULT_FAIL
        r.html_result = "<p>Test Failed.</p><p>"+impactout+"</p>"
        print "fail"
        print impactout
    
result_service = rospy.ServiceProxy('test_result', TestResult)

rospy.sleep(2);

# block until the test_result service is available
rospy.wait_for_service('test_result')
result_service.call(r)

