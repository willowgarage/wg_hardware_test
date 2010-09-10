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
This tests that a single dropped EtherCAT packet triggers a warning.
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
IGNORE_TIME = 5

def _ecat_diag(pkts = 0):
    array = DiagnosticArray()
    stat = DiagnosticStatus()
    stat.name = 'EtherCAT Master'
    stat.level = 0
    stat.message = 'OK'
    stat.values.append(KeyValue('Dropped Packets', str(pkts)))

    array.header.stamp = rospy.get_rostime()
    array.status.append(stat)
    
    return array

class TestDroppedPacket(unittest.TestCase):
    def __init__(self, *args):
        super(TestDroppedPacket, self).__init__(*args)

        self._mutex = threading.Lock()
        rospy.init_node('test_monitor_drop_pkt')
        self._ok = True
        self._level = 0

        self._start = rospy.get_time()
        self._last_msg = None
        self._max_lvl = -1

        self._halted = False

        self._snapped = False
        self._snapshot_sub = rospy.Subscriber('snapshot_trigger', std_msgs.msg.Empty, self._snap_cb)

        self._halt_srv = rospy.Service('pr2_etherCAT/halt_motors', Empty, self.on_halt)
        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)

        self._reset_test = rospy.ServiceProxy('reset_test', Empty)

        rospy.Subscriber('test_status', TestStatus, self._cb)

        self.motors_pub = rospy.Publisher('pr2_etherCAT/motors_halted', Bool)
        self.cal_pub = rospy.Publisher('calibrated', Bool, latch=True)

    def on_halt(self, srv):
        pass

    def _snap_cb(self, msg):
        self._snapped = True

    def _cb(self, msg):
        with self._mutex:
            if not self._last_msg:
                self._start = rospy.get_time()

            self._last_msg = msg

            if rospy.get_time() - self._start > IGNORE_TIME:
                self._max_lvl = max(msg.test_ok, self._max_lvl)

    def test_drop_pkt(self):
        self.cal_pub.publish(True)
        while not rospy.is_shutdown():
            self._diag_pub.publish(_ecat_diag())
            self.motors_pub.publish(False)
            sleep(1.0)
            if rospy.get_time() - self._start > GRACE_TIME:
                break

        # Publish single dropped packet
        self._diag_pub.publish(_ecat_diag(2))
        self.motors_pub.publish(False)
        sleep(1.0)

        # Publish same status for 10 seconds
        for i in range(0, 10):
            self._diag_pub.publish(_ecat_diag(2))
            self.motors_pub.publish(False)
            sleep(1.0)

        with self._mutex:
            self.assert_(not rospy.is_shutdown(), "Rospy shutdown")

            self.assert_(self._last_msg is not None, "No data from test monitor")
            
            # Check that message level is OK
            self.assert_(self._last_msg.test_ok == TestStatus.RUNNING, "Test monitor reports that we're not running. Level: %d. Message: %s" % (self._last_msg.test_ok, self._last_msg.message))

            # Check that we went into warning state
            self.assert_(self._max_lvl == TestStatus.WARNING, "We didn't get warning message from the ecat listener. Max level: %d" % self._max_lvl)

            # Check that snapshot trigger was called
            self.assert_(not self._snapped, "Snapshot trigger was called, but we didn't halt")

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '-v':
        suite = unittest.TestSuite()
        suite.addTest(TestDroppedPacket('test_dropped_pkt'))

        unittest.TextTestRunner(verbosity=2).run(suite)
    else:
        rostest.run(PKG, sys.argv[0], TestDroppedPacket, sys.argv)
