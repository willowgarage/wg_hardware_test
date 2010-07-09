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
##\brief Listens to diagnostics from wge100 camera and reports OK/FAIL


from __future__ import with_statement

PKG = 'pr2_hardware_test_monitor'
import roslib; roslib.load_manifest(PKG)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import *

import rospy

import threading

from pr2_hw_listener import PR2HWListenerBase

STALE_TIMEOUT = 8.0
ERROR_TIMEOUT = 8.0
ERROR_MAX = 3

class CameraListener(PR2HWListenerBase):
    def __init__(self):
        self._diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, self._diag_callback)
        self._mutex = threading.Lock()

        self._lvl = 3 # No updates
        self._update_time = 0

        # Record last good message and the time
        self._last_ok_time = rospy.get_time()
        self._error_cnt = 0 # Increment on error, reset on OK. Leave value on warning
        self._last_msg_ok = False # True if last value was OK

        self._was_stale = True # Warn if we go stale
        self._reported_vals = False # Log whether we've reported any errors

    def _diag_callback(self, msg):
        with self._mutex:
            has_wge100 = False
            this_lvl = 0
            this_msg = ''
            for stat in msg.status:
                if stat.name.find('wge100') >= 0:
                    this_lvl = max(stat.level, this_lvl)
                    self._update_time = rospy.get_time()
                    has_wge100 = True
                    
                    if stat.level == this_lvl:
                        this_msg = stat.message

            if has_wge100:
                if this_lvl == 0:
                    self._last_ok_time = rospy.get_time()
                    self._error_cnt = 0

                    # Don't reset last_msg value until we're reported it
                    if self._reported_vals:
                        self._last_msg_ok = True
                else:
                    self._last_msg_ok = False

                if this_lvl > 1: # Received error value
                    rospy.logwarn('Camera error from wge100 camera. Message: %s' % this_msg)
                    self._error_cnt += 1

                self._was_stale = False
                self._reported_vals = False

    
    def check_ok(self):
        with self._mutex:
            msg = 'OK'
            lvl = 0 

            # Report a warning if we saw a warning or error in last message
            if not self._last_msg_ok:
                lvl = 1
                msg = 'Camera Warning'

            # Report error if we've seen multiple errors, or gone too long without good message
            if self._error_cnt > ERROR_MAX or rospy.get_time() - self._last_ok_time > ERROR_TIMEOUT:
                lvl = 2
                msg = 'Camera Error'
                if rospy.get_time() - self._last_ok_time > ERROR_TIMEOUT:
                    rospy.logwarn('Too long since last OK message received. Camera error')

            if rospy.get_time() - self._update_time > STALE_TIMEOUT:
                if not self._was_stale:
                    rospy.logerr('wge100 camera is stale. No updates from camera')
                self._was_stale = True

                lvl = 3
                msg = 'Camera Stale'
                if self._update_time == 0:
                    msg = 'No Camera Data'
        
            # We've reported everything, so we can reset ourselves
            self._reported_vals = True

        return lvl, msg, None
    
