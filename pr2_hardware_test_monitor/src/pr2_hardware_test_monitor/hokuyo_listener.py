#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
##\brief Listens to diagnostics from hokuyo_node and reports OK/FAIL

PKG = 'pr2_hardware_test_monitor'

import roslib
roslib.load_manifest(PKG)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import *

import rospy

import threading

from pr2_hw_listener import PR2HWListenerBase

class HokuyoListener(PR2HWListenerBase):
    """
    Listens to diagnostics from a Hokuyo laser scanner with the given namespace.
    """
    def __init__(self):
        self._diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, self._diag_callback)
        self._mutex = threading.Lock()

        self._ok = True
        self._update_time = 0
        self.name = 'tilt_hokuyo_node'

    def create(self, params):
        """
        params must have the value "name", which is the namespace of the Hokuyo to listen to.

        \param params { } : ROS Parameters in namespace
        \return bool : True if created OK
        """
        if not params.has_key('name'):
            rospy.logerr('Hokuyo Listener was not given param "name"')
            return False
        self.name = params['name']
        
        return True

    def _diag_callback(self, msg):
        self._mutex.acquire()
        for stat in msg.status:
            if stat.name.find(self.name) >= 0:
                self._ok = (stat.level == 0)
                self._update_time = rospy.get_time()
                if not self._ok:
                    break

        self._mutex.release()
    
    def check_ok(self):
        self._mutex.acquire()
        msg = ''
        stat = 0
        if not self._ok:
            stat = 2
            msg = 'Hokuyo Error'

        if rospy.get_time() - self._update_time > 3:
            stat = 3
            msg = 'Hokuyo Stale'
            if self._update_time == 0:
                msg = 'No Hokuyo Data'
        
        self._mutex.release()
        return stat, msg, None
    
