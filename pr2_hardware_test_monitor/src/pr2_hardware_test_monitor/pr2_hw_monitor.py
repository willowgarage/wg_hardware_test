#!/usr/bin/env python
#
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
##\brief Loads listeners, monitors status of PR2 hardware tests

from __future__ import with_statement

PKG = 'pr2_hardware_test_monitor'
import roslib
roslib.load_manifest(PKG)

import rospy

from pr2_self_test_msgs.msg import TestStatus

from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import *
import std_msgs.msg

import traceback
import threading
import sys

HEARTBEAT_TIMEOUT = 60 # If no heartbeat, shut down
IGNORE_TIME = 30 # Allow errors for the first few seconds (grace period)

def create_listener(params, listeners):
    """
    @brief Loads listener from parameters
    
    @param params {} : Must have "type", "file". "pkg" is optional
    @param listeners [] : Newly created listener is appended 
    @return bool : True if listener created successfully
    """
    if not (params.has_key('type') and params.has_key('file')):
        rospy.logerr('Params "type" and "file" weren\'t found!')
        return False
    
    file = params['file']
    type = params['type']
    ##\todo Fix this
    #pkg = params['pkg'] if params.has_key('pkg') else PKG
    pkg = PKG

    try:    
        import_str = '%s.%s' % (pkg, file)
        __import__(import_str)
        pypkg = sys.modules[import_str]
        listener_type = getattr(pypkg, type)
    except Exception, e:
        rospy.logerr('Couldn\'t load listener %s from %s.%s.\n\nException: %s' % (type, pkg, file, traceback.format_exc()))
        return False
    
    try:
        listener = listener_type()
    except Exception, e:
        rospy.logerr('Listener %s failed to construct.\nException: %s' % (type, traceback.format_exc()))
        return False
                     
    if not listener.create(params):
        return False

    listeners.append(listener)
    return True

def _make_message(level, warnings, errors):
    """
    Write status message with warnings and errors
    """
    if level == 0:
        return 'OK'
    if level == TestStatus.WARNING:
        return ', '.join(warnings)
    if level > TestStatus.WARNING:
        if len(errors + warnings) > 0:
            return ', '.join(errors + warnings)
        else:
            return 'Error'

class TestMonitor:
    """
    TestMonitor class loads listeners, and polls them to check for status updates on the test

    """
    def __init__(self):
        self._listeners = []

        self._mutex = threading.Lock()

        my_params = rospy.get_param("~")

        self._listeners_ok = True

        for ns, listener_param in my_params.iteritems():
            if not create_listener(listener_param, self._listeners):
                rospy.logerr('Listener failed to initialize. Namespace: %s' % ns)
                self._listeners_ok = False

        # Clear initial state of test monitor
        self._reset_state()

        self._snapshot_pub = rospy.Publisher('snapshot_trigger', std_msgs.msg.Empty)

        self._status_pub = rospy.Publisher('test_status', TestStatus)
        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)

        self.reset_srv = rospy.Service('reset_test', Empty, self._reset_test)
        self.halt_srv = rospy.Service('halt_test', Empty, self._halt_test)

        self._heartbeat_time = rospy.get_time()
        self._heartbeat_sub = rospy.Subscriber('/heartbeat', std_msgs.msg.Empty, self._on_heartbeat)


    @property
    def init_ok(self):
        """
        Returns whether the monitor initialized correctly. Used in unit tests.
        """
        return len(self._listeners) > 0 and self._listeners_ok

    def _on_heartbeat(self, msg):
        self._heartbeat_time = rospy.get_time()

    def _reset_state(self):
        """
        Reset state of monitor for startup or on reset
        """
        self._heartbeat_halted = False
        self._was_ok = True
        self._start_time = rospy.get_time()

        # Clear error/warning conditions
        self._errors = []
        self._latched_lvl = TestStatus.RUNNING

    def _reset_test(self, srv):
        with self._mutex:
            self._reset_state()

            for listener in self._listeners:
                try:
                    listener.reset()
                except Exception, e:
                    rospy.logerr('Listener failed to reset!')

        return EmptyResponse()

    def _halt_test(self, srv):
        with self._mutex:
            self._halt_listeners()

        return EmptyResponse()

    def _halt_listeners(self):
        for listener in self._listeners:
            try:
                listener.halt()
            except Exception, e:
                rospy.logerr('Listener failed to halt!')

        self._snapshot_pub.publish()

    def _check_status(self):
        level = TestStatus.RUNNING
        array = DiagnosticArray()
        warnings = []
        errors = []

        for listener in self._listeners:
            try:
                lvl, msg, diags = listener.check_ok()
            except Exception, e:
                rospy.logerr('Listener failed to check status. %s' % traceback.format_exc())
                lvl, msg, diags = (TestStatus.ERROR, 'Listener Error', None)

            level = max(level, lvl)
            if msg is not None and msg != '':
                if lvl == TestStatus.WARNING:
                    warnings.append(msg)
                if lvl > TestStatus.WARNING:
                    errors.append(msg)
            if diags is not None:
                array.status.extend(diags)
        
        if len(self._listeners) == 0:
            level = TestStatus.STALE
            return level, 'No listeners', array

        if not self._listeners_ok:
            level = TestStatus.ERROR
            errors = [ 'Listener Startup Error' ]

        # Check if grace period
        grace_period = rospy.get_time() - self._start_time < IGNORE_TIME

        # Halt for any errors received
        if not grace_period and \
                (self._was_ok and level > TestStatus.WARNING):
            self._halt_listeners()
            rospy.logerr('Halted test after failure. Failure message: %s' % ', '.join(errors))
            self._was_ok = False
            
        if not self._heartbeat_halted and rospy.get_time() - self._heartbeat_time > HEARTBEAT_TIMEOUT:
            rospy.logerr('No heartbeat from Test Manager received, halting test')
            self._halt_listeners()
            self._heartbeat_halted = True

        if self._heartbeat_halted:
            level = TestStatus.STALE
            errors = [ 'No heartbeat' ]

        # Latch all error levels
        if not grace_period and level > TestStatus.WARNING:
            self._latched_lvl = max(self._latched_lvl, level)
            
        # Report the max of our current level and the latch
        level = max(self._latched_lvl, level)

        # Latch all error messages
        if not grace_period and len(errors) > 0:
            for err in errors:
                if not err in self._errors:
                    self._errors.append(err)

        if not grace_period:
            # Make message based on latched status
            message = _make_message(level, warnings, self._errors)
        else:
            # Make message based on our current status
            message = _make_message(level, warnings, errors)

        return level, message, array



    def publish_status(self):
        """
        Called at 1Hz. Polls listeners, publishes diagnostics, test_status.
        """
        level, message, array = self._check_status()

        if len(array.status) > 0:
            array.header.stamp = rospy.get_rostime()
            self._diag_pub.publish(array)

        test_stat = TestStatus()
        test_stat.test_ok = int(level)
        test_stat.message = message

        self._status_pub.publish(test_stat)
