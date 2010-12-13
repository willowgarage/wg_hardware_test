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

##\brief Listens to transmissions of specified joints, halts motors if error detected.

from __future__ import with_statement
PKG = 'pr2_hardware_test_monitor'

import roslib
roslib.load_manifest(PKG)

from pr2_mechanism_msgs.msg import MechanismStatistics
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import *
from std_msgs.msg import Bool

import rospy

import threading

from pr2_hw_listener import PR2HWListenerBase

GRACE_HITS = 5 # Max number of errors before halt motors called

TURRET_NAME = 'fl_caster_rotation_joint'
L_WHEEL_NAME = 'fl_caster_l_wheel_joint'
R_WHEEL_NAME = 'fl_caster_r_wheel_joint'

# From URDF
WHEEL_RADIUS = 0.074792
WHEEL_OFFSET = 0.049

ALLOWED_SLIP = 0.020 # (2.0cm/interval)
UPDATE_INTERVAL = 0.25 

class CasterPosition(object):
    def __init__(self, msg):
        self.turret = None
        self.l_wheel = None
        self.r_wheel = None
        self.turret_cal = False

        if msg is None:
            return

        for js in msg:
            if js.name == TURRET_NAME:
                self.turret_cal = js.is_calibrated
                self.turret = js.position
            if js.name == L_WHEEL_NAME:
                self.l_wheel = js.position
            if js.name == R_WHEEL_NAME:
                self.r_wheel = js.position
    
    def has_data(self):
        return self.turret_cal and self.turret is not None and self.l_wheel is not None and self.r_wheel is not None

def check_position(new, old):
    if not new.has_data() or not old.has_data():
        return True, None, None

    # Calculate turret rotation
    turret = new.turret - old.turret
    
    # Calculate wheel travel from offset
    wheel_dx = turret * WHEEL_OFFSET

    # Distances wheels actually moved
    r_dx = (new.r_wheel - old.r_wheel) * WHEEL_RADIUS
    l_dx = -1 * (new.l_wheel - old.l_wheel) * WHEEL_RADIUS

    # Error
    r_err = r_dx - wheel_dx
    l_err = l_dx - wheel_dx

    if (abs(r_err) > ALLOWED_SLIP) or (abs(l_err) > ALLOWED_SLIP):
        return False, r_err, l_err
    return True, r_err, l_err

##\brief Makes sure caster doesn't slip or drive forward
class CasterSlipListener(object):
    def __init__(self):
        self._ok = True
        self._update_time = 0
                
        self.last_position = CasterPosition(None)

        self._max_l_err_pos = None
        self._max_r_err_pos = None

        self._max_l_err_neg = None
        self._max_r_err_neg = None

        self._max_l_err_pos_reset = None
        self._max_r_err_pos_reset = None

        self._max_l_err_neg_reset = None
        self._max_r_err_neg_reset = None

        self._num_errors = 0
        self._num_errors_since_reset = 0

        self.diag = DiagnosticStatus()
        self.stat = 0
        self.msg = ''
            
    def create(self, params):
        return True

    def reset(self):
        self._ok = True
        self._num_errors_since_reset = 0

        self._max_l_err_pos_reset = None
        self._max_r_err_pos_reset = None

        self._max_l_err_neg_reset = None
        self._max_r_err_neg_reset = None

    def update(self, msg):
        if rospy.get_time() - self._update_time < UPDATE_INTERVAL:
            return self._ok

        self._update_time = rospy.get_time()

        position = CasterPosition(msg.joint_statistics)

        ok, r_err, l_err = check_position(position, self.last_position)
        if not ok:
            self._ok = False
            self._num_errors += 1
            self._num_errors_since_reset += 1
        
        if r_err is None:
            pass
        elif r_err > 0:
            self._max_r_err_pos = max(self._max_r_err_pos, abs(r_err))
            self._max_r_err_pos_reset = max(self._max_r_err_pos_reset, abs(r_err))
        else:
            self._max_r_err_neg = max(self._max_r_err_neg, abs(r_err))
            self._max_r_err_neg_reset = max(self._max_r_err_neg_reset, abs(r_err))

        if l_err is None:
            pass
        elif l_err > 0:
            self._max_l_err_pos = max(self._max_l_err_pos, abs(l_err))
            self._max_l_err_pos_reset = max(self._max_l_err_pos_reset, abs(l_err))
        else:
            self._max_l_err_neg = max(self._max_l_err_neg, abs(l_err))
            self._max_l_err_neg_reset = max(self._max_l_err_neg_reset, abs(l_err))

        self.last_position = position

        return self._ok

    def get_status(self):
        stat = 0
        if not self._ok:
            stat = 2

        if rospy.get_time() - self._update_time > 3:
            stat = 3
                
        diag = DiagnosticStatus()
        diag.level = stat
        diag.name = 'Caster Slip Listener'
        diag.message = 'OK'
        if diag.level == 2:
            diag.message = 'Caster Slipping'
        if diag.level == 3:
            diag.message = 'Caster Stale'
            diag.level = 2

        diag.values.append(KeyValue("Turret", str(TURRET_NAME)))
        diag.values.append(KeyValue("R Wheel", str(R_WHEEL_NAME)))
        diag.values.append(KeyValue("L Wheel", str(L_WHEEL_NAME)))
        diag.values.append(KeyValue("Turret Position", str(self.last_position.turret)))
        diag.values.append(KeyValue("R Wheel Position", str(self.last_position.r_wheel)))
        diag.values.append(KeyValue("L Wheel Position", str(self.last_position.l_wheel)))
        diag.values.append(KeyValue("Max Pos. Left Slip", str(self._max_l_err_pos)))
        diag.values.append(KeyValue("Max Neg. Left Slip", str(self._max_l_err_neg)))
        diag.values.append(KeyValue("Max Pos. Right Slip", str(self._max_r_err_pos)))
        diag.values.append(KeyValue("Max Neg. Right Slip", str(self._max_r_err_neg)))
        diag.values.append(KeyValue("Max Pos. Left Slip (Reset)", str(self._max_l_err_pos_reset)))
        diag.values.append(KeyValue("Max Neg. Left Slip (Reset)", str(self._max_l_err_neg_reset)))
        diag.values.append(KeyValue("Max Pos. Right Slip (Reset)", str(self._max_r_err_pos_reset)))
        diag.values.append(KeyValue("Max Neg. Right Slip (Reset)", str(self._max_r_err_neg_reset)))
        diag.values.append(KeyValue("Wheel Offset", str(WHEEL_OFFSET)))
        diag.values.append(KeyValue("Wheel Diameter", str(WHEEL_RADIUS)))
        diag.values.append(KeyValue("Allowed Slip", str(ALLOWED_SLIP)))
        diag.values.append(KeyValue("Update Interval", str(UPDATE_INTERVAL)))
        diag.values.append(KeyValue("Total Errors", str(self._num_errors)))
        diag.values.append(KeyValue("Errors Since Reset", str(self._num_errors_since_reset)))

        return diag


##\brief Loads individual joint listeners, monitors all robot transmissions
class TransmissionListener(PR2HWListenerBase):
    def __init__(self):
        self._joint_monitors = []
        self._mech_sub = None
        self._halt_motors = rospy.ServiceProxy('pr2_etherCAT/halt_motors', Empty)

        self._trans_sub = rospy.Subscriber('pr2_transmission_check/transmission_status', Bool, self._trans_cb)
        self._reset_trans = rospy.ServiceProxy('pr2_transmission_check/reset_trans_check', Empty)

        self._mutex = threading.Lock()
        self._ok = True # Status callback OK
        self._jms_ok = True # Joint monitors OK
        self._last_msg_time = 0
        
    def create(self, params):
        for joint, joint_param in params.iteritems():
            # Ignore setup params
            if joint == 'type' or joint == 'file':
                continue

            # NOTE: Not creating JointTransmissionListeners because pr2_transmission_check
            # can do this for us.
            if joint == 'caster_slip':
                joint_mon = CasterSlipListener()
                if not joint_mon.create(joint_param):
                    rospy.logerr('Unable to create CasterSlipListener')
                    return False
                self._joint_monitors.append(joint_mon)

        if self._joint_monitors:
            self._mech_sub = rospy.Subscriber('mechanism_statistics', MechanismStatistics, self._callback)

        return True

    def _trans_cb(self, msg):
        with self._mutex:
            self._last_msg_time = rospy.get_time()

            was_ok = self._ok
            self._ok = msg.data

            if not self._ok and was_ok:
                rospy.logerr('Halting motors, broken transmission.')
                try:
                    self._halt_motors()
                except Exception, e:
                    import traceback
                    rospy.logerr('Caught exception trying to halt motors: %s', traceback.format_exc())


        
    def _callback(self, msg):
        with self._mutex:
            self._last_msg_time = rospy.get_time()
            
            was_ok = self._jms_ok
            
            for joint_mon in self._joint_monitors:
                ok = joint_mon.update(msg)
                self._ok_jms = ok and self._ok_jms
                

        # Halt if broken
        if not self._jms_ok and was_ok:
            rospy.logerr('Halting motors, caster slipping')
            try:
                self._halt_motors()
            except Exception, e:
                import traceback
                rospy.logerr('Caught exception trying to halt motors: %s', traceback.format_exc())

    def reset(self):
        with self._mutex:
            self._ok = True
            self._jms_ok = True
            for joint_mon in self._joint_monitors:
                joint_mon.reset()

            try:
                self._reset_trans()
            except Exception, e:
                rospy.logerr("Unable to reset tranmission checker")
            
    def check_ok(self):
        with self._mutex:
            if self._last_msg_time == 0:
                return 3, "No trans status", None
            if rospy.get_time() - self._last_msg_time > 3:
                return 3, "Trans status stale", None

            if self._ok and self._jms_ok:
                status = DiagnosticStatus.OK
                msg = ''
            elif self._ok:  
                status = DiagnosticStatus.ERROR
                msg = 'Caster Slipping'
            else:
                status = DiagnosticStatus.ERROR
                msg = 'Transmission Broken'
        
            diag_stats = []
            for joint_mon in self._joint_monitors:
                diag_stats.append(joint_mon.get_status())
                
        return status, msg, diag_stats
