#!/usr/bin/env python
#
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#

##\author Kevin Watts
##\brief Checks if DMM is available for projector test

PKG = 'qualification'

import roslib
roslib.load_manifest(PKG)

from pr2_self_test_msgs.srv import ScriptDone, ScriptDoneRequest

import rospy
import traceback
import socket
import subprocess


class ProjectorChecker:
    def __init__(self):
        self._dmm_address = rospy.get_param("~dmm_address")

    @property
    def dmm_address(self): return self._dmm_address

    def check_projector(self):
        retcode = subprocess.call('ping -c1 -W1 %s > /dev/null' % self._dmm_address, shell=True)
        if retcode != 0:
            return False

        try :
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            lxi_port = 5025
            sock.settimeout(15)
            sock.connect((self._dmm_address, 5025))
            sock.close()
            return True
        except socket.error:
            return False
        except Exception, e:
            rospy.logerr('Caught unknown exception checking projector:\n%s' % traceback.format_exc())
            return False
        return True
        

def main():
    r = ScriptDoneRequest()
    r.result = ScriptDoneRequest.RESULT_ERROR
    r.failure_msg = 'Unable to contact projector'

    try:
        rospy.init_node('projector_monitor')
        done_srv = rospy.ServiceProxy('prestartup_done', ScriptDone)

        monitor = ProjectorChecker()
        if not monitor.check_projector():
            r.failure_msg = 'Unable to contact projector at %s' % monitor.dmm_address
            done_srv.call(r)
            rospy.spin()
        else:
            r.result = ScriptDoneRequest.RESULT_OK
            r.failure_msg = 'Projector OK'
            done_srv.call(r)
        
    except Exception, e:
        import traceback
        r.failure_msg = 'Error contacting projector. Exception:\n%s' % traceback.format_exc()
        done_srv.call(r)
        
    rospy.spin()

if __name__ == '__main__':
    main()
