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
##\brief Tests that test monitor latches error state

"""
This tests that a transient error status, such as a camera error, produces 
the correct behavior. We expect that an error, after a "grace period",
will cause the "halt" method of all listeners to be called, and that the error 
state will "latch" until reset is called.
"""

from __future__ import with_statement

PKG = 'pr2_hardware_test_monitor'
import roslib; roslib.load_manifest(PKG)

import unittest
import rospy, rostest
from time import sleep
import sys

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pr2_self_test_msgs.msg import TestStatus
from std_msgs.msg import Bool
import std_msgs.msg
from std_srvs.srv import *
import threading

GRACE_TIME = 35 # 30 + 5 seconds for HW monitor to be ready

def _camera_diag(level = 0):
    array = DiagnosticArray()
    stat = DiagnosticStatus()
    stat.name = 'wge100: Driver Status'
    stat.level = level
    stat.message = 'OK'

    motor_stat = DiagnosticStatus()
    motor_stat.name = 'EtherCAT Master'
    motor_stat.level = 0
    motor_stat.values = [
        KeyValue(key='Dropped Packets', value='0'),
        KeyValue(key='RX Late Packet', value='0')]

    mcb_stat = DiagnosticStatus()
    mcb_stat.name = 'EtherCAT Device (my_motor)'
    mcb_stat.level = 0
    mcb_stat.values.append(KeyValue('Num encoder_errors', '0'))

    array.header.stamp = rospy.get_rostime()
    array.status.append(stat)
    array.status.append(motor_stat)
    array.status.append(mcb_stat)
        
    return array

class TestMonitorLatch(unittest.TestCase):
    def __init__(self, *args):
        super(TestMonitorLatch, self).__init__(*args)

        self._mutex = threading.Lock()
        rospy.init_node('test_monitor_latch_test')
        self._ignore_time = 5
        self._start_time = rospy.get_time()
        self._ok = True
        self._level = 0

        self._start = rospy.get_time()
        self._last_msg = None

        self._halted = False

        # Publish that we're calibrated
        self._cal_pub = rospy.Publisher('calibrated', Bool, latch=True)
        self._cal_pub.publish(True)

        self._snapped = False
        self._snapshot_sub = rospy.Subscriber('snapshot_trigger', std_msgs.msg.Empty, self._snap_cb)

        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)
        self._pub = rospy.Publisher('pr2_etherCAT/motors_halted', Bool)
        self._hlt = rospy.Service('pr2_etherCAT/halt_motors', Empty, self._hlt_cb)
        self._rst = rospy.Service('pr2_etherCAT/reset_motors', Empty, self._rst_cb)

        self._reset_test = rospy.ServiceProxy('reset_test', Empty)

        rospy.Subscriber('test_status', TestStatus, self._cb)

    def _snap_cb(self, msg):
        self._snapped = True

    def _hlt_cb(self, srv):
        self._halted = True
        return EmptyResponse()

    def _rst_cb(self, srv):
        self._halted = False
        return EmptyResponse()
    
    def _cb(self, msg):
        with self._mutex:
            if not self._last_msg:
                self._start = rospy.get_time()

            self._last_msg = msg


    def test_monitor(self):
        while not rospy.is_shutdown():
            self._pub.publish(False)
            self._diag_pub.publish(_camera_diag())
            sleep(1.0)
            if rospy.get_time() - self._start > GRACE_TIME:
                break

        # Publish camera error for 10 seconds
        for i in range(0, 10):
            self._pub.publish(False)
            self._diag_pub.publish(_camera_diag(level = 2))
            sleep(1.0)

        # Publish cameras OK 5x
        for i in range(0, 5):
            self._pub.publish(False)
            self._diag_pub.publish(_camera_diag())
            sleep(1.0)

        with self._mutex:
            self.assert_(not rospy.is_shutdown(), "Rospy shutdown")

            self.assert_(self._last_msg is not None, "No data from test monitor")
            
            # Check that message level is error state
            self.assert_(self._last_msg.test_ok == TestStatus.ERROR, "Didn't record error state from Test Monitor. Should have error state because motors halted. Level: %d" % self._last_msg.test_ok)

            # Check that message message has motors data
            self.assert_(self._last_msg.message != 'OK', "Got OK message from test monitor, even with error level")
            self.assert_(self._last_msg.message.find("Camera Error") > -1, "Didn't get camera error message")

            # Check that it called halt_motors correctly
            self.assert_(self._halted, "Halt motors wasn't called after failure")

            # Check that snapshot trigger was called
            self.assert_(self._snapped, "Snapshot trigger wasn't called after halt")

        # Reset the test and make sure we're OK
        self._reset_test()

        # Publish good data for 5s
        for i in range(0, 5):
            self._pub.publish(False)
            self._diag_pub.publish(_camera_diag())
            sleep(1.0)

        with self._mutex:
            self.assert_(not rospy.is_shutdown(), "Rospy shutdown")
            self.assert_(self._last_msg.test_ok == TestStatus.RUNNING, "Test didn't reset properly after error condition")

            self.assert_(not self._halted, "Reset motors wasn't called after reset")

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '-v':
        suite = unittest.TestSuite()
        suite.addTest(TestMonitorLatch('test_monitor'))

        unittest.TextTestRunner(verbosity=2).run(suite)
    else:
        rostest.run(PKG, sys.argv[0], TestMonitorLatch, sys.argv)
