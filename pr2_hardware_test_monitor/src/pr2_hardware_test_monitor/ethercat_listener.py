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
import re
import itertools

from pr2_hw_listener import PR2HWListenerBase

from collections import deque

DROPS_PER_HOUR = 10

ENCODER_ERRORS_FIELD = 'Num encoder_errors'

class EthercatListener(PR2HWListenerBase):
    def __init__(self):
        self._mutex = threading.Lock()

        self._cal = False
        self._ok = True
        self._update_time = -1
        self._diag_update_time = -1

        # Records dropped packets. New drops appended to deque
        # Deque is kept =< max length of self._drops_per_hour
        self._dropped_times = deque()

        self._net_drops = 0

        self._encoder_errors_detected = False
        self._last_encoder_errors_update = rospy.get_time()
        # Stores encoder errors by motor for each motor
        self._encoder_errors_cnt = {}

        self._drops_per_hour = DROPS_PER_HOUR

        # A list of expected device names, or None if we don't care
        self._expected_devices = None
        self._incorrect_devices = None
        self._device_name_re = re.compile('EtherCAT Device \((.*)\).*')
        
        self._params = []

    def create(self, params):
       # Give it 10 seconds to start up
        try:
            rospy.wait_for_service('pr2_etherCAT/halt_motors', 10)
            self._srv_ok = True
        except Exception, e:
            self._srv_ok = False
            rospy.logerr('Unable to find halt motors service. Unable to initialize ethercat listener')
            return False

        if params.has_key('drops_per_hour'):            
            self._drops_per_hour = params['drops_per_hour'] 

        if params.has_key('expected_devices'):
            self._expected_devices = params['expected_devices']
            
        self._params = params

        self._reset_motors = rospy.ServiceProxy('pr2_etherCAT/reset_motors', Empty)

        self._halt_motors = rospy.ServiceProxy('pr2_etherCAT/halt_motors', Empty)

        # Make this persistent in case the master goes down
        self._halt_motors2 = rospy.ServiceProxy('pr2_etherCAT/halt_motors', Empty, persistent = True)

        self._ecat_sub = rospy.Subscriber('pr2_etherCAT/motors_halted', Bool, self._motors_cb)

        self._cal_sub = rospy.Subscriber('calibrated', Bool, self._cal_cb)

        self._diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, self._diag_cb)

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

        with self._mutex:
            self._dropped_times.clear()
            self._encoder_errors_detected = False

    def _update_drops(self, stat, now):
        if stat.name != 'EtherCAT Master':
            raise Exception('Diagnostic status with invalid name! Expected \"EtherCAT Master\", got: %s',
                            stat.name)

        drops = -1
        lates = -1
        for kv in stat.values:
            if kv.key == 'Dropped Packets':
                drops = int(kv.value)
            elif kv.key == 'RX Late Packet':
                lates = int(kv.value)

        if (drops == -1 or lates == -1):
            raise Exception("Diagnostics didn't contain data for dropped or late packets. Drops: %d. Lates: %d" %
                            (drops, lates))

        # For every new dropped packet, we add the current timestamp to our deque
        new_net_drops = drops - lates
        if new_net_drops > self._net_drops:
            for i in range(min(new_net_drops - self._net_drops, self._drops_per_hour)):
                self._dropped_times.append(now)

        self._net_drops = new_net_drops
        
        # Clean up the buffer to the max_length
        while len(self._dropped_times) > self._drops_per_hour:
            self._dropped_times.popleft()

        self._diag_update_time = rospy.get_time()

    def _update_encoder_errors(self, stat):
        """
        Check for encoder errors for every motor. 
        
        Updates cache of encoder errors. Reports encoder error detected if
        it can't find the number of encoder errors, or if encoder errors count
        increases.
        """
        if not stat.name.startswith('EtherCAT Device ('):
            raise Exception("Invalid diagnostic status name: %s" % stat.name)

        # Ignore led_projector, no encoder errors
        if stat.name.find('led_projector') > -1:
            return

        last_errors = self._encoder_errors_cnt.get(stat.name, 0)
        
        curr_errors = -1
        for kv in stat.values:
            if kv.key == ENCODER_ERRORS_FIELD:
                curr_errors = int(kv.value)

        if curr_errors < 0:
            rospy.logerr('Unable to find encoder errors for motor %s', stat.name)
            self._encoder_errors_detected = True
            return 
               
        self._last_encoder_errors_update = rospy.get_time()

        if curr_errors > last_errors:
            self._encoder_errors_detected = True
            
        # Update cache with last value for encoder status
        self._encoder_errors_cnt[stat.name] = curr_errors
        
        

    def _diag_cb(self, msg):
        is_ethercat_diag_msg = False
        devices = []
        with self._mutex:
            now = msg.header.stamp.to_sec()
            for stat in msg.status:
                if stat.name == 'EtherCAT Master':
                    self._update_drops(stat, now)
                    is_ethercat_diag_msg = True
                elif stat.name.startswith('EtherCAT Device ('):
                    self._update_encoder_errors(stat)
                    m = self._device_name_re.match(stat.name)
                    if m:
                        device_name = m.group(1)
                        devices.append(device_name)

        if is_ethercat_diag_msg and (self._expected_devices is not None):
            # check list of present devices against expected list of devices
            if self._expected_devices != devices:
                with self._mutex:
                    self._incorrect_devices = devices

    def _cal_cb(self, msg):
        with self._mutex:
            self._cal = msg.data

    def _motors_cb(self, msg):
        with self._mutex:
            self._ok = not msg.data
            self._update_time = rospy.get_time()
    
    def _is_dropping_pkts(self):
        """
        Check if we're dropping packets. 
        A drop is true if we've had more than 10 dropped packets in last hour.
         
        @return bool : True if dropping packets
        """
        now = rospy.get_time()

        if len(self._dropped_times) < self._drops_per_hour:
            return False

        return abs(now - self._dropped_times[0]) < 3600
            

    def check_ok(self):
        msg = ''
        stat = 0
        with self._mutex:
            if not self._cal:
                stat = 1
                msg = 'Uncalibrated'

            if not self._ok:
                stat = 2
                msg = 'Motors Halted'

            if self._incorrect_devices is not None:
                stat = 2
                msg = 'Wrong devices'
                dev_list = itertools.izip_longest(self._expected_devices, self._incorrect_devices, fillvalue ='<NOTHING>')
                for (expected_dev, incorrect_dev) in dev_list:
                    if expected_dev != incorrect_dev:
                        msg = 'Expected %s, found %s' % (expected_dev, incorrect_dev)
                        break

            # Error if we've had dropped packets
            if self._is_dropping_pkts():
                stat = 2
                msg = 'Dropping Packets'

            # Encoder errors check, #4814
            if self._encoder_errors_detected:
                stat = 2
                msg = 'Encoder Errors'

            if rospy.get_time() - self._last_encoder_errors_update > 3.0:
                stat = 3
                msg = 'No MCB Encoder Status'

            if rospy.get_time() - self._diag_update_time > 3.0:
                stat = 3
                msg = 'No MCB Diagnostics'

            if rospy.get_time() - self._update_time > 3.0:
                stat = 3
                msg = 'EtherCAT Stale'
                if self._update_time == -1:
                    msg = 'No EtherCAT Data'
        
        return stat, msg, []
    
