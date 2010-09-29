#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

##\author Kevin Watts
##\brief Check to make sure Xilinx is installed on machine

PKG = 'qualification'
import roslib
roslib.load_manifest(PKG)

from pr2_self_test_msgs.srv import ScriptDone, ScriptDoneRequest

import rospy 

NAME = 'check_for_Xilinx'

import os
import sys


if __name__ == "__main__":
    rospy.init_node(NAME)
    
    ip = rospy.myargv()[1]
    via = rospy.myargv()[2]

    r = ScriptDoneRequest()

    success = False

    rv = os.path.exists('/opt/Xilinx/11.1/settings32.sh')

    if rv:
        r.result = ScriptDoneRequest.RESULT_OK
        r.failure_msg = "Xilinx installed"
    else:
        r.result = ScriptDoneRequest.RESULT_ERROR
        r.failure_msg = "Xilinx isn't installed correctly. You will not be able to run this test."
    
    # block until the test_result service is available
    rospy.wait_for_service('prestartup_done')
    result_service = rospy.ServiceProxy('prestartup_done', ScriptDone)
    result_service.call(r)

    rospy.spin()
