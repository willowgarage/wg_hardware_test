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
#

##\author Kevin Watts

##\brief Tests receipt of test monitor messages from life tests

from __future__ import with_statement

DURATION = 10

PKG = 'pr2_hardware_test_monitor'
import roslib; roslib.load_manifest(PKG)

import unittest
import rospy, rostest
from time import sleep
import sys
from optparse import OptionParser
import math

from std_srvs.srv import Empty, EmptyResponse
from pr2_mechanism_msgs.msg import MechanismStatistics, JointStatistics, ActuatorStatistics
from pr2_self_test_msgs.msg import TestStatus
import threading

class TransListenerErrorTest(unittest.TestCase):
    def __init__(self, *args):
        super(TransListenerErrorTest, self).__init__(*args)

        self._mutex = threading.Lock()
        rospy.init_node('test_trans_listener_error')
        self._ignore_time = 5 # Ignore values for about 5 seconds
        self._start_time = rospy.get_time()
        self._ok = True
        self._message = None
        self._level = 0

        self._roll_error = rospy.get_param("~roll_error", False)
        self._flex_error = rospy.get_param("~flex_error", False)

        self._start_time = rospy.get_time()

 
        rospy.Subscriber('test_status', TestStatus, self.cb)

        self.mech_pub = rospy.Publisher('mechanism_statistics', MechanismStatistics)
        self._reset_srv = rospy.Service('pr2_etherCAT/reset_motors', Empty, self.on_reset)
        self._halt_srv = rospy.Service('pr2_etherCAT/halt_motors', Empty, self.on_halt)

    def on_halt(self, srv):
        with self._mutex:
            self._ok = False
        return EmptyResponse()

    def on_reset(self, srv):
        with self._mutex:
            self._ok = True
        return EmptyResponse()

    def publish_mech_stats(self, flex_error = False, roll_error = False):
        # Joint state is a sine, period 1s, Amplitude 2,
        trig_arg = rospy.get_time() - self._start_time

        sine = math.sin(trig_arg)
        cosine = math.cos(trig_arg)

        jnt_st = JointStatistics()
        jnt_st.name = 'r_elbow_flex_joint'
        jnt_st.position = float(1 * sine) - 1.2
        jnt_st.is_calibrated = 1

        cont_st = JointStatistics()
        cont_st.name = 'r_forearm_roll_joint'
        cont_st.position = 5 * float(0.5 * sine)
        cont_st.velocity = 2.5 * float(0.5 * cosine)
        cont_st.is_calibrated = 1

        cont_act_st = ActuatorStatistics()
        cont_act_st.name = 'r_forearm_roll_motor'
        cont_act_st.calibration_reading = False 
        wrapped_position = (cont_st.position % 6.28)
        if wrapped_position < 3.14:
            cont_act_st.calibration_reading = True
        if roll_error:
            cont_act_st.calibration_reading = not cont_act_st.calibration_reading

        act_st = ActuatorStatistics()
        act_st.name = 'r_elbow_flex_motor'
        act_st.calibration_reading = True
        if jnt_st.position > -1.1606:
            act_st.calibration_reading = False
        if flex_error:
            act_st.calibration_reading = not act_st.calibration_reading

        mech_st = MechanismStatistics()
        mech_st.actuator_statistics = [ act_st, cont_act_st ]
        mech_st.joint_statistics = [ jnt_st, cont_st ]
        mech_st.header.stamp = rospy.get_rostime()

        self.mech_pub.publish(mech_st)
    
    def cb(self, msg):
        if rospy.get_time() - self._start_time < self._ignore_time:
            return

        with self._mutex:
            self._message = msg.message
            self._level = msg.test_ok

    def test_monitor(self):
        while not rospy.is_shutdown():
            sleep(0.1)
            self.publish_mech_stats()
            if rospy.get_time() - self._start_time > DURATION:
                break
        rospy.loginfo('Sent some normal data')
        with self._mutex:
            self.assert_(self._message, "No data from test monitor")
            self.assert_(self._ok, "Not OK after sending only good data. Message: %s" % self._message)

        for i in range(0, 10): # 10 consecutive errors
            self.publish_mech_stats(self._flex_error, self._roll_error)
            sleep(0.1)

        for i in range(0, 10): # Some OK values
            self.publish_mech_stats()
            sleep(0.1)
                
        with self._mutex:
            self.assert_(not rospy.is_shutdown(), "Rospy shutdown")
            self.assert_(self._message, "No data from test monitor")
            self.assert_(self._level == 2, "Transmission listener didn't report error. Level: %d. Message: %s" % (self._level, self._message))
            self.assert_(not self._ok, "Transmission listener didn't call halt motors!")


if __name__ == '__main__':
    print 'SYS ARGS:', sys.argv
    rostest.run(PKG, sys.argv[0], TransListenerErrorTest, sys.argv)
