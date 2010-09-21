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
from pr2_self_test_msgs.srv import *
import subprocess
import select

if __name__ == '__main__':
    rospy.init_node("test_caller", anonymous=True)
    args = rospy.myargv()

    r = TestResultRequest()
    r.plots = []
    
    try:
        if len(args) < 3:
            raise Exception('Usage: ./test_caller.py <pkg> <program> <args>')
        
        popen_args = ['rosrun']+args[1:]

        p = subprocess.Popen(popen_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        fdmap = {p.stdout:sys.stdout, p.stderr:sys.stderr}
        output = ""
        while fdmap and not rospy.is_shutdown():
            fdl = select.select(fdmap.keys(), [], [])
            for fd in fdl[0]:
                read = fd.read(1)
                if read:
                    fdmap[fd].write(read)
                else:
                    del fdmap[fd]
                output += read
    except Exception, e:
        r.text_summary = "Failed with exception in test_caller.py"
        r.result = TestResultRequest.RESULT_FAIL
        r.html_result = "<p>Test Failed with exception.</p><p>%s</p>"%str(e)
        output = str(e)
    else:
        p.wait()
        retcode = p.returncode
        
        output = output.replace('\n','<br>')
    
        if not p.returncode:
            r.text_summary = "Successful completion of args[0]."
            r.html_result = "<p>Test passed.</p><p>"+output+"</p>" 
            r.result = TestResultRequest.RESULT_PASS
            print "pass"
        else:
            r.text_summary = "Completion of args[0] with error code %i."%retcode
            r.result = TestResultRequest.RESULT_FAIL
            r.html_result = "<p>Test Failed.</p><p>"+output+"</p>"
            print "fail"
    finally:
        try:
            p.kill()
        except OSError, e:
            if e.errno != 3:
                raise
    
    result_service = rospy.ServiceProxy('test_result', TestResult)
    rospy.wait_for_service('test_result')
    result_service.call(r)
