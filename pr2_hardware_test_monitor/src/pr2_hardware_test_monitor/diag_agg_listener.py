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
##\brief Listens to diagnostics_agg and reports statusx


from __future__ import with_statement

PKG = 'pr2_hardware_test_monitor'
import roslib; roslib.load_manifest(PKG)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import *

import rospy

import threading
import types

from robot_monitor.robot_monitor_panel import State

from pr2_hw_listener import PR2HWListenerBase

class DiagAggState(State):
    def __init__(self, whitelist = None, ignore_categories = None):
        """
        \param whitelist [ str ] : Values to focus on.
        \param ignore_categories [ str ] : Ignore matching values
        """
        State.__init__(self)

        self._whitelist = whitelist 
        self._ignore_categories = ignore_categories if ignore_categories else []
              
    def _is_ignored(self, name):
        """
        Check if item should be ignored.
        If we have a whitelist, False if item in it. 
        Else if, True if we're supposed to ignore it.
        """
        if self._whitelist:
            if name in self._whitelist:
                return False
            elif name.lstrip('/') in self._whitelist:
                return False
            else:
                return True

        for ignore in self._ignore_categories:
            if name == ignore:
                return True
            if '/' + ignore == name:
                return True

        return False


    def get_top_level_state(self):
        """
        Get top level status (OK, WARN, ERROR, STALE) of diagnostics
        Handles ignore categories, etc.
        """
        level = -1
        min_level = 255
        msgs = []
        
        if len(self.get_items()) == 0:
            return level
    
        for item in self.get_items().itervalues():
            # Only look at "top level" items
            if self.get_parent(item) is not None:
                continue
            
            if self._is_ignored(item.status.name):
                continue

            if item.status.level > level:
                level = item.status.level
                if item.status.level < min_level:
                    min_level = item.status.level

            if item.status.level > 0:
                msgs.append(item.status.name.lstrip('/'))
                
        # Top level is error if we have stale items, unless all stale
        if level > 2 and min_level <= 2:
            level = 2
            
        return level, msgs

def _convert_to_list(val):
    if type(val) in (list, tuple):
        return val
    else:
        return [ str(val) ]

class DiagAggListener(PR2HWListenerBase):
    """
    Listens to /diagnostics_agg topic and checks top level state of diagnostics
    Will latch any error messages until reset occurs

    """
    def __init__(self):
        self._mutex = threading.Lock()

        self._level = 0
        self._msgs = []
        self._update_time = 0

    def create(self, params):
        """
        Parameter values:
         * ignore_diags : Any values to ignore when computing top level state
         * whitelist : Only focus on these values 
        Both may be string or list. 

        \param params { str : str } : ROS parameters to initialize 
        \return bool : True if init OK
        """
        if params.has_key('ignore_diags'):
            ignore_diags = _convert_to_list(params['ignore_diags'])
        else:
            ignore_diags = [ 'Other' ]

        whitelist = None
        if params.has_key('whitelist'):
            whitelist = _convert_to_list(params['whitelist'])
        
        self._state = DiagAggState(whitelist = whitelist, ignore_categories = ignore_diags)

        self._diag_agg_sub = rospy.Subscriber('/diagnostics_agg', DiagnosticArray, self._diag_callback)

        return True

    def reset(self):
        """
        Clears state of messages that were in error or warning
        """
        self._msgs = []

    def _diag_callback(self, msg):
        with self._mutex:
            self._update_time = rospy.get_time()
            (added, removed, all) = self._state.update(msg)

            self._level, msgs = self._state.get_top_level_state()

            for msg in msgs:
                if self._msgs.count(msg) < 1:
                    self._msgs.append(msg)
            
    def check_ok(self):
        """
        @return (int, str, None) : Level, Message. No diagnostics
        """
        with self._mutex:
            msg = ''
            stat = self._level
            if stat == 1:
                msg = 'Diag Warn: %s' % (', '.join(self._msgs))
            if stat > 1:
                msg = 'Diag Error: %s' % (', '.join(self._msgs))

            if rospy.get_time() - self._update_time > 3:
                stat = 3
                msg = 'Diagnostics Stale'
                if self._update_time == 0:
                    msg = 'No Diagnostics Data'
        
        return stat, msg, None
    
