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
##\brief Listens to pr2_etherCAT/motors_halted, makes sure etherCAT state is OK

from __future__ import with_statement
PKG = 'pr2_hardware_test_monitor'

import roslib; roslib.load_manifest(PKG)

from std_msgs.msg import Bool
from std_srvs.srv import *
from diagnostic_msgs.msg import DiagnosticArray

import rospy

import threading

from pr2_hw_listener import PR2HWListenerBase

class EthercatListener(PR2HWListenerBase):
    def __init__(self):
        self._mutex = threading.Lock()

        self._cal = False
        self._ok = True
        self._update_time = -1

    def create(self, params):
       # Give it 10 seconds to start up
        try:
            rospy.wait_for_service('pr2_etherCAT/halt_motors', 10)
            self._srv_ok = True
        except Exception, e:
            self._srv_ok = False
            rospy.logerr('Unable to find halt motors service. Unable to initialize ethercat listener')
            return False

        self._reset_motors = rospy.ServiceProxy('pr2_etherCAT/reset_motors', Empty)

        self._halt_motors = rospy.ServiceProxy('pr2_etherCAT/halt_motors', Empty)

        # Make this persistent in case the master goes down
        self._halt_motors2 = rospy.ServiceProxy('pr2_etherCAT/halt_motors', Empty, persistent = True)

        self._ecat_sub = rospy.Subscriber('pr2_etherCAT/motors_halted', Bool, self._motors_cb)

        self._cal_sub = rospy.Subscriber('calibrated', Bool, self._cal_cb)

        self._diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, self._diag_cb)

        self._dropped_cnt = 0
        self._last_drop_time = 0 # Use rospy.get_time()

        return True

    # Try twice to halt motors, using persistant service for one try
    def halt(self):
        try:
            self._halt_motors2()
            self._halt_motors()

        except Exception, e:
            try:
                # Redo, with new persistent service
                self._halt_motors2 = rospy.ServiceProxy('pr2_etherCAT/halt_motors', Empty, persistent = True)
                self._halt_motors2()

                return
            except Exception, e:
                rospy.logerr('Unable to halt motors. pr2_etherCAT may have died')
                

    def reset(self):
        try:
            self._reset_motors()
        except Exception, e:
            rospy.logerr('Unable to reset motors. pr2_etherCAT may have died')

    def _diag_cb(self, msg):
        with self._mutex:
            for stat in msg.status:
                if stat.name == 'EtherCAT Master':
                    for kv in stat.values:
                        if kv.key == 'Dropped Packets':
                            if not unicode(kv.value).isnumeric():
                                self._last_drop_time = rospy.get_time()
                                rospy.logwarn('Unable to convert %s into integer. Unable to count dropped packets' % kv.value)
                                break

                            curr_drops = int(kv.value)
                            if curr_drops > self._dropped_cnt:
                                self._last_drop_time = rospy.get_time()
                                
                            self._dropped_cnt = curr_drops
                            break


    def _cal_cb(self, msg):
        with self._mutex:
            self._cal = msg.data

    def _motors_cb(self, msg):
        with self._mutex:
            self._ok = not msg.data
            self._update_time = rospy.get_time()
    
    def check_ok(self):
        with self._mutex:
            msg = ''
            stat = 0
            if not self._cal:
                stat = 1
                msg = 'Uncalibrated'

            # Warn if we've had a dropped packets in the last three seconds
            if rospy.get_time() - self._last_drop_time < 3:
                stat = 1
                msg = 'Dropping Packets'

            if not self._ok:
                stat = 2
                msg = 'Motors Halted'

            if rospy.get_time() - self._update_time > 3:
                stat = 3
                msg = 'EtherCAT Stale'
                if self._update_time == -1:
                    msg = 'No EtherCAT Data'
        
        return stat, msg, []
    
