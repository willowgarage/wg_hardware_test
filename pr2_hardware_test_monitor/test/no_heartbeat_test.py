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
This tests that not publishing a 'heartbeat' topic will cause the test monitor
to halt everything.
"""

from __future__ import with_statement

DURATION = 120
SHORT = 10

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

class TestMonitorHeartbeat(unittest.TestCase):
    def __init__(self, *args):
        super(TestMonitorHeartbeat, self).__init__(*args)

        self._mutex = threading.Lock()
        rospy.init_node('test_monitor_no_beat_test')
        self._ignore_time = 5
        self._start_time = rospy.get_time()
        self._ok = True
        self._level = 0

        self._start = rospy.get_time()
        self._last_msg = None

        self._halted = False

        self._heartbeat_pub = rospy.Publisher('/heartbeat', std_msgs.msg.Empty)

        # Publish that we're calibrated
        self._cal_pub = rospy.Publisher('calibrated', Bool, latch=True)
        self._cal_pub.publish(True)

        # Snapshot trigger 
        self._snapped = False
        self._snapshot_sub = rospy.Subscriber('snapshot_trigger', std_msgs.msg.Empty, self._snap_cb)

        # Publish camera/motors data
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
        # Publish good data for a bit
        while not rospy.is_shutdown():
            self._pub.publish(False)
            self._diag_pub.publish(_camera_diag())
            sleep(1.0)
            if rospy.get_time() - self._start > SHORT:
                break

        # Check that our data is good for now
        with self._mutex:
            self.assert_(not rospy.is_shutdown(), "Rospy shutdown")
            self.assert_(self._last_msg is not None, "No data from test monitor")
            self.assert_(self._last_msg.test_ok == TestStatus.RUNNING, "Should have good data, since we're publishing OK")
            self.assert_(not self._halted, "Motors halted, but we should be OK")

        # Keep publishing good data until we trip the heartbeat
        while not rospy.is_shutdown() and self._last_msg.test_ok == TestStatus.RUNNING:
            self._pub.publish(False)
            self._diag_pub.publish(_camera_diag())
            sleep(1.0)
            if rospy.get_time() - self._start > DURATION:
                break

        with self._mutex:
            self.assert_(not rospy.is_shutdown(), "Rospy shutdown")
            self.assert_(self._last_msg is not None, "No data from test monitor")

            # Check that we're halted for heartbeat
            self.assert_(self._last_msg.test_ok == TestStatus.STALE, "Didn't record stale state from monitor. Should have stale state because no heartbeat")

            # Check that it called halt_motors correctly
            self.assert_(self._halted, "Halt motors wasn't called after no heartbeat")

            self.assert_(self._snapped, "Snapshot trigger wasn't called for halt")


        # Send the heartbeat to reset the monitor automatically, #4878
        for i in range(5):
            if rospy.is_shutdown():
                break

            self._pub.publish(False)
            self._diag_pub.publish(_camera_diag())
            self._heartbeat_pub.publish()
            sleep(1.0)

        with self._mutex:
            self.assert_(not rospy.is_shutdown(), "Rospy shutdown")
            self.assert_(self._last_msg is not None, "No data from test monitor")

            # Check that we're halted for heartbeat
            self.assert_(self._last_msg.test_ok == TestStatus.RUNNING, "Test monitor should have reset automatically after heartbeat sent")



if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '-v':
        suite = unittest.TestSuite()
        suite.addTest(TestMonitorHeartbeat('test_monitor'))

        unittest.TextTestRunner(verbosity=2).run(suite)
    else:
        rostest.run(PKG, sys.argv[0], TestMonitorHeartbeat, sys.argv)
