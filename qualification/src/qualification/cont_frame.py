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

##\author Kevin Watts
##\brief Allows qualification test to be run continuously

from __future__ import with_statement

PKG = 'qualification'

import roslib
roslib.load_manifest(PKG)

import rospy

import os
import sys
import threading

from datetime import datetime # May want to change to rostime
import wx
import time
from wx import xrc
from wx import html

from xml.dom import minidom

import result_dir
import csv

class ContinuousTestFrame(wx.Frame):
    def __init__(self, parent):
        wx.Frame.__init__(self, parent, wx.ID_ANY, "Continuous Testing")

        self._manager = parent

        # Load the XRC resource
        xrc_path = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'xrc/gui.xrc')
        self._res = xrc.XmlResource(xrc_path)

        # Load the main panel
        self._cont_panel = self._res.LoadPanel(self, 'continuous_test_panel')

        # Input
        self._submit_box = xrc.XRCCTRL(self._cont_panel, 'submit_box')
        self._abort_button = xrc.XRCCTRL(self._cont_panel, 'abort_button')
        self._abort_button.Bind(wx.EVT_BUTTON, self.abort)

        # Display
        self._complete_text = xrc.XRCCTRL(self._cont_panel, 'completed_text')
        self._passed_text = xrc.XRCCTRL(self._cont_panel, 'passed_text')
        self._last_result_text = xrc.XRCCTRL(self._cont_panel, 'last_result_text')

        self._last_result = 'N/A'
        self._total_tests = 0
        self._passed_tests = 0

        date_str = datetime.now().strftime("%m%d%Y_%H%M")
        basename = '%s_cycle_log.csv' % date_str
        self._log = os.path.join(result_dir.RESULTS_DIR, basename.replace(' ', ''))
        with open(self._log, 'wb') as f:
            log_csv = csv.writer(f)
            log_csv.writerow(['Time', 'Result', 'Tar Name', 'Submit OK'])


    def abort(self, event):
        are_you_sure = wx.MessageDialog(self, "Are you sure you want to abort continuous testing?",
                                        "Confirm Abort", wx.OK|wx.CANCEL)
        if are_you_sure.ShowModal() != wx.ID_OK:
            return

        wx.MessageBox('Continuous mode disabled. Data in %s.' % self._log, 'Continuous Mode Finished', 
                      wx.OK|wx.ICON_ERROR, self)

        self._manager.stop_continuous_testing()

    def new_results(self, results, invent):
        self._total_tests += 1
        if results.get_pass_bool():
            self._passed_tests += 1
        self._last_result = results.get_test_result_str()

        # Get tar filename, write to summary file
        self._results.write_results_to_file() # Write to temp dir   
        tar_name = results.tar_name

        submit_stat = 'N/A'
        if self._submit_box.IsChecked() and invent:
            res, msg = results.log_results(invent)
            if not res:
                submit_stat = 'FAIL: %s' % msg
            else:
                submit_stat = 'OK'

        with open(self._log, 'ab') as f:
            log_csv = csv.writer(f)
            log_csv.writerow([datetime.now().strftime("%m/%d/%Y %H:%M:%S"), self._last_result, tar_name, submit_stat])

                                       
        self._complete_text.SetValue(str(self._total_tests))
        self._passed_text.SetValue(str(self._passed_tests))
        self._last_result_text.SetValue(str(self._last_result))
