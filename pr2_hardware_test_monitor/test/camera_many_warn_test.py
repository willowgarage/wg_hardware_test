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
This tests that many warning/error codes from the wge100 camera are reported as an error.
"""

from __future__ import with_statement

PKG = 'pr2_hardware_test_monitor'
import roslib; roslib.load_manifest(PKG)

import unittest
import rospy, rostest
from time import sleep
import sys

import random
random.seed()

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
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

    array.header.stamp = rospy.get_rostime()
    array.status.append(stat)
    
    return array

class TestCameraWarn(unittest.TestCase):
    def __init__(self, *args):
        super(TestCameraWarn, self).__init__(*args)

        self._mutex = threading.Lock()
        rospy.init_node('test_monitor_latch_test')
        self._ignore_time = 5
        self._start_time = rospy.get_time()
        self._ok = True
        self._level = 0

        self._start = rospy.get_time()
        self._last_msg = None
        self._max_lvl = -1

        self._halted = False

        self._snapped = False
        self._snapshot_sub = rospy.Subscriber('snapshot_trigger', std_msgs.msg.Empty, self._snap_cb)

        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)

        rospy.Subscriber('test_status', TestStatus, self._cb)

    def _snap_cb(self, msg):
        self._snapped = True

    def _cb(self, msg):
        with self._mutex:
            if not self._last_msg:
                self._start = rospy.get_time()

            self._last_msg = msg

            if rospy.get_time() - self._start_time > self._ignore_time:
                self._max_lvl = max(msg.test_ok, self._max_lvl)


    def test_many_cam_warn(self):
        while not rospy.is_shutdown():
            self._diag_pub.publish(_camera_diag())
            sleep(1.0)
            if rospy.get_time() - self._start > GRACE_TIME:
                break

        # Publish lots of camera warnings/errors
        for i in range(0, 15):
            self._diag_pub.publish(_camera_diag(level = random.randint(1, 2)))
            sleep(1.0)

        # Publish cameras OK a few times
        for i in range(0, 10):
            self._diag_pub.publish(_camera_diag())
            sleep(1.0)

        with self._mutex:
            self.assert_(not rospy.is_shutdown(), "Rospy shutdown")

            self.assert_(self._last_msg is not None, "No data from test monitor")
            
            # Check that message level is error state
            self.assert_(self._last_msg.test_ok == TestStatus.ERROR, "Test monitor should be in error state. Level: %d" % self._last_msg.test_ok)
            # Check that snapshot trigger was called
            self.assert_(self._snapped, "Snapshot trigger wasn't called")

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '-v':
        suite = unittest.TestSuite()
        suite.addTest(TestCameraWarn('test_many_cam_warn'))

        unittest.TextTestRunner(verbosity=2).run(suite)
    else:
        rostest.run(PKG, sys.argv[0], TestCameraWarn, sys.argv)
