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
##\brief Panel for starting, stopping and logging life tests 

from __future__ import with_statement
PKG = 'life_test'

import roslib
roslib.load_manifest(PKG)

import sys, os, math, string
import csv
import traceback
from time import sleep, strftime, localtime
import threading
import socket, subprocess


import wx
from wx import xrc

import rospy

from std_srvs.srv import * # Empty

# Stuff from life_test package
from pr2_self_test_msgs.msg import TestStatus, TestInfo

from test_param import TestParam, LifeTest
from test_record import TestRecord
from writing_core import *

from runtime_monitor.monitor_panel import MonitorPanel
from roslaunch_caller import roslaunch_caller 

class TestMonitorPanel(wx.Panel):
    def __init__(self, parent, manager, test, serial):
        wx.Panel.__init__(self, parent)

        self._manager = manager

        self._mutex = threading.Lock()

        self._status_sub = None
        
        # Set up test and loggers
        self._test = test
        self._serial = serial
        self._record = TestRecord(test, serial)

        # Test Data panel
        xrc_path = os.path.join(roslib.packages.get_pkg_dir('life_test'), 'xrc/gui.xrc')

        self._panel = xrc.XmlResource(xrc_path).LoadPanel(self, 'test_panel')
        self._test_desc = xrc.XRCCTRL(self._panel, 'test_desc')
        self._test_desc.SetValue(self._test.desc)

        self._launch_button = xrc.XRCCTRL(self._panel, 'launch_test_button')
        self._launch_button.Bind(wx.EVT_BUTTON, self.start_stop_test)
        self._launch_button.SetToolTip(wx.ToolTip("Start and stop the test"))

        self._test_bay_ctrl = xrc.XRCCTRL(self._panel, 'test_bay_ctrl')
        self._test_bay_ctrl.SetItems(self._manager.room.get_bay_names(self._test.needs_power))
        self._test_bay_ctrl.SetToolTip(wx.ToolTip("Select location of test"))
        
        # Set default start time based on test
        self._end_cond_type = xrc.XRCCTRL(self._panel, 'end_cond_type')
        self._end_cond_type_label = xrc.XRCCTRL(self._panel, 'duration_label')
        self._test_duration_ctrl = xrc.XRCCTRL(self._panel, 'test_duration_ctrl')

        if self._test.duration > 0:
            self._end_cond_type.SetStringSelection('Hours')
            self._test_duration_ctrl.SetRange(0, max(168, self._test.duration)) # Week
            self._test_duration_ctrl.SetValue(int(self._test.duration))
        else:
            self._end_cond_type.SetStringSelection('Continuous')
            self._test_duration_ctrl.SetRange(0, 0) # Can't change limits
            self._test_duration_ctrl.SetValue(0)

        # User sets Continuous, Hours, Minutes
        self._end_cond_type.Bind(wx.EVT_CHOICE, self.on_end_choice)
        self._end_cond_type.SetToolTip(wx.ToolTip("Select stop time"))
        
        # Close panel down
        self._close_button = xrc.XRCCTRL(self._panel, 'close_button')
        self._close_button.Bind(wx.EVT_BUTTON, self.on_close)
        self._close_button.SetToolTip(wx.ToolTip("Close this test panel"))
        
        self._status_bar = xrc.XRCCTRL(self._panel, 'test_status_bar')
        self._status_bar.SetToolTip(wx.ToolTip("Current status of this test"))

        # Pause and reset buttons
        self._reset_button = xrc.XRCCTRL(self._panel, 'reset_motors_button')
        self._reset_button.Bind(wx.EVT_BUTTON, self.on_reset_test)
        self._reset_button.SetToolTip(wx.ToolTip("Resume test operation"))

        self._halt_button = xrc.XRCCTRL(self._panel, 'halt_motors_button')
        self._halt_button.Bind(wx.EVT_BUTTON, self.on_halt_test)
        self._reset_button.SetToolTip(wx.ToolTip("Pause test"))

        # Logs and operator input
        self._user_log = xrc.XRCCTRL(self._panel, 'user_log_input')
        self._user_log.SetToolTip(wx.ToolTip("Enter any user notes here"))
        self._user_submit = xrc.XRCCTRL(self._panel, 'user_submit_button')
        self._user_submit.Bind(wx.EVT_BUTTON, self.on_user_entry)
        self._user_submit.SetToolTip(wx.ToolTip("Submit entry to test log"))

        self._done_time_ctrl = xrc.XRCCTRL(self._panel, 'done_time_ctrl')
        self._done_time_ctrl.SetToolTip(wx.ToolTip("Remaining time to completion"))
        
        self._elapsed_time_ctrl = xrc.XRCCTRL(self._panel, 'elapsed_time_ctrl')
        self._elapsed_time_ctrl.SetToolTip(wx.ToolTip("Time since test was opened"))
        self._active_time_ctrl = xrc.XRCCTRL(self._panel, 'active_time_ctrl')
        self._active_time_ctrl.SetToolTip(wx.ToolTip("Total operating time of test"))

        self._log_ctrl = xrc.XRCCTRL(self._panel, 'test_log')
        self._test_log_window = xrc.XRCCTRL(self._panel, 'test_log_window')

        # Power controls
        self._power_board_text = xrc.XRCCTRL(self._panel, 'power_board_text')
        self._power_board_text.SetBackgroundColour("White")
        self._power_board_text.SetValue("Test Not Running")

        self._estop_status = xrc.XRCCTRL(self._panel, 'estop_status')
        self._estop_status.SetValue("Test Not Running")

        self._power_run_button = xrc.XRCCTRL(self._panel, 'power_run_button')
        self._power_run_button.Bind(wx.EVT_BUTTON, self.on_power_run)
        self._power_run_button.SetToolTip(wx.ToolTip("Turn on power"))

        self._power_standby_button = xrc.XRCCTRL(self._panel, 'power_standby_button')
        self._power_standby_button.Bind(wx.EVT_BUTTON, self.on_power_standby)
        self._power_run_button.SetToolTip(wx.ToolTip("Turn power to standby"))

        self._power_disable_button = xrc.XRCCTRL(self._panel, 'power_disable_button')
        self._power_disable_button.Bind(wx.EVT_BUTTON, self.on_power_disable)
        self._power_run_button.SetToolTip(wx.ToolTip("Turn power to disable"))        

        # Bay data
        self._power_sn_text = xrc.XRCCTRL(self._panel, 'power_sn_text')
        self._power_breaker_text = xrc.XRCCTRL(self._panel, 'power_breaker_text')
        self._machine_text = xrc.XRCCTRL(self._panel, 'machine_text')

        # Add runtime to the panel...
        self._notebook = xrc.XRCCTRL(self._panel, 'test_data_notebook')
        wx.CallAfter(self._create_monitor)

        self._sizer = wx.BoxSizer(wx.HORIZONTAL)
        self._sizer.Add(self._panel, 1, wx.EXPAND)
        self.SetSizer(self._sizer)
        self.Layout()
        
        self._bay = None
        self._current_log = {}
        self._diag_msgs = {}

        self._is_running = False
        self._stat_level = 127 # Not launched
        self._test_msg = 'None'
        self._power_stat = "No data"
        self._estop_stat = False
        self._stop_count = 0

        # Launches test, call stop to kill it
        self._test_launcher = None

        # Test log data
        self._test_complete = False

        # Timeout for etherCAT diagnostics, starts when test launched
        self.timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_timer, self.timer)
        self._last_message_time = rospy.get_time()
        self._timeout_interval = 30.0
        self._is_stale = True

        # Timeout for powerboard status, starts if power comes up
        self.power_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_power_timer, self.power_timer)
        self.last_power_update = rospy.get_time()
        self.power_timeout = 5.0
        
        # Timer for invent logging
        self.invent_timer = wx.Timer(self)
        self.Bind(wx.EVT_TIMER, self.on_invent_timer, self.invent_timer)
        self.invent_timeout = 600
        self.invent_timer.Start(self.invent_timeout * 500)
        self._is_invent_stale = True

        self.update_controls()
        self._enable_controls()

    def _create_monitor(self):
        """
        Loads runtime_monitor panel. Called after to make display work
        """
        self._monitor_panel = MonitorPanel(self._notebook, 'empty_topic')
        self._monitor_panel.SetSize(wx.Size(400, 500))
        self._notebook.AddPage(self._monitor_panel, "Diagnostics")
        self._notebook.ChangeSelection(0)

    @property
    def launched(self):
        return self._test_launcher is not None

    def on_end_choice(self, event = None):
        choice = self._end_cond_type.GetStringSelection()

        # Set test_duration_ctrl units also
        label = choice.lower()
        if choice == 'Continuous':
            label = 'N/A'
        self._end_cond_type_label.SetLabel(choice.lower())

        # Set spin ctrl based on end type
        self._test_duration_ctrl.SetValue(0)
        active_time = self._record.get_cum_time()

        if choice == 'Hours':
            hrs = math.ceil((active_time / 3600))
            self._test_duration_ctrl.SetRange(hrs, 168) # Week
            self._test_duration_ctrl.SetValue(hrs)
        elif choice == 'Minutes':
            min = math.ceil((active_time / 60))
            self._test_duration_ctrl.SetRange(min, 600) # 10 Hrs
            self._test_duration_ctrl.SetValue(min + 10)
        else:
            self._test_duration_ctrl.SetRange(0, 0) # Can't change limits
            self._test_duration_ctrl.SetValue(0)

    def on_close(self, event):
        try:
            self.update_test_record('Closing down test.')
            self.record_test_log()

            if self._bay is not None:
                self._manager.test_stop(self._bay)
        except:
            rospy.logerr('Exception on close: %s' % traceback.format_exc())

        if self.launched:
            self.stop_test()

        if self._status_sub is not None:
            self._status_sub.unregister()
            self._status_sub = None

        self._manager.close_tab(self._serial)

    def on_user_entry(self, event):
        entry = self._user_log.GetValue()
        msg = 'OPERATOR: ' + entry
        self.update_test_record(msg)
        self._user_log.Clear()
        self._user_log.SetFocus()

    ##\brief Updates record, notifies operator if necessary
    def update_test_record(self, note = ''):
        alert, msg = self._record.update(self.launched, self._is_running, self._is_stale, 
                                         note, self._test_msg)
        message = msg + ' ' + note

        if alert > 0 or note != '':
            self._current_log[rospy.get_time()] = message
            self._display_logs()


    ##\brief Displays logs on screen. Only update_test_record should be called
    def _display_logs(self):
        kys = dict.keys(self._current_log)
        kys.sort()

        log_str = ''
        for ky in kys:
            log_str += strftime("%m/%d/%Y %H:%M:%S: ", 
                                localtime(ky)) + self._current_log[ky] + '\n'

        self._log_ctrl.AppendText(log_str)

        # Test log control in second panel
        self._test_log_window.Freeze()
        (x, y) = self._test_log_window.GetViewStart()
        self._test_log_window.SetPage(self._record.write_log())
        self._test_log_window.Scroll(x, y)
        self._test_log_window.Thaw()

        self._current_log = {}

    ##\todo Private
    def calc_run_time(self):
        end_condition = self._end_cond_type.GetStringSelection()
        
        duration = self._test_duration_ctrl.GetValue()

        if end_condition == 'Hours':
            return duration * 3600
        if end_condition == 'Minutes':
            return duration * 60
        if end_condition == 'Seconds':
            return duration
        else: #if end_condition == 'Continuous':
            return 10**10 # Roughly 300 years

    ##\todo Private
    def calc_remaining(self):
        total_sec = self.calc_run_time()
        cum_sec = self._record.get_cum_time()

        return total_sec - cum_sec
        
        
    def start_timer(self):
        self.timer.Start(1000 * self._timeout_interval, True)
        
    def on_timer(self, event):
        if not self.launched:
            return

        interval = rospy.get_time() - self._last_message_time
        
        was_stale = self._is_stale

        if interval > 300:  # 300 second timeout before we mark stale
            # Make EtherCAT status stale
            self._is_running = False
            self._is_stale = True

            # Halt test if it goes stale, #4007
            if not was_stale:
                self.on_halt_test()
                rospy.logerr('Halting test on machine %s. Update is stale for %d seconds' % (self._bay.name, int(interval)))
                self.update_test_record('Halted test after no data received for %d seconds.' % int(interval))

            self.update_controls(4)
            self.update_test_record()
            self.stop_if_done()
        else:
            self._is_stale = False

    def on_status_check(self):
        msg = TestInfo()
        msg.serial = str(self._serial)
        msg.test_name = str(self._test.short)
        if self.launched:
            msg.test_status = int(self._stat_level)
            msg.bay_name = str(self._bay.name)
            msg.machine = str(self._bay.machine)
            if self._bay.board is not None and self._test.needs_power:
                msg.board = self._bay.board
                msg.breaker = self._bay.breaker
                msg.power_status = self._power_stat
                if self._estop_stat:
                    msg.estop = 1
                else:
                    msg.estop = 0
            else:
                msg.board = 0
                msg.breaker = 0
                                
        else:
            msg.test_status = 127
            msg.bay_name = "None"
            msg.board = 0
            msg.breaker = 0
            msg.machine = ""

        msg.elapsed = self._record.get_cum_time()
        msg.status_msg = self._test_msg
        
        return msg        

    def _test_power_cmd(self):
        if not self.launched:
            wx.MessageBox('Test is not launched. Unable to command power board', 'Test is not launched', wx.OK|wx.ICON_ERROR, self)
            return False

        if self._bay.board is None:
            wx.MessageBox('Test bay has no power board. Unable to command power board', 'No Power Board',
                          wx.OK|wx.ICON_ERROR, self)
            return False
        return True        

    def on_power_run(self, event):
        self.update_test_record("Turning on power.")
        if not self._test_power_cmd():
            return

        if not self._manager.power_run(self._bay):
            self.update_test_record("Unable to command power board.")
            wx.MessageBox('Unable to run power board. Board: %d, breaker: %d' % (self._bay.board, self._bay.breaker), wx.OK|wx.ICON_ERROR, self)

    def on_power_standby(self, event):
        self.update_test_record("Setting power to standby.")
        if not self._test_power_cmd():
            return
        
        if not self._manager.power_standby(self._bay):
            self.update_test_record("Unable to command power board.")
            wx.MessageBox('Unable to standby power board. Board: %d, breaker: %d' % (self._bay.board, self._bay.breaker), wx.OK|wx.ICON_ERROR, self)

    def on_power_disable(self, event):
        self.update_test_record("Setting power to disable.")
        if not self._test_power_cmd():
            return

        if not self._manager.power_disable(self._bay):
            self.update_test_record("Unable to command power board.")
            wx.MessageBox('Unable to disable power board. Board: %d, breaker: %d' % (self._bay.board, self._bay.breaker), wx.OK|wx.ICON_ERROR, self)

    def start_power_timer(self):
        self.power_timer.Start(1000 * self.power_timeout, True)

    def on_power_timer(self, event = None):
        if rospy.get_time() - self.last_power_update > self.power_timeout:
            self.update_board("Stale", False)

    ##\brief Updates power board control with status
    def update_board(self, value, estop):
        if not self.launched:
            return

        self._power_stat = value
        self._estop_stat = estop

        self.start_power_timer()

        if value == "Stale":
            self._power_board_text.SetBackgroundColour("Light Blue")
            self._power_board_text.SetValue("Stale")
            self._estop_status.SetBackgroundColour("Light Blue")
            self._estop_status.SetValue("E-Stop Stale")
            return

        elif value == "Standby":
            self._power_board_text.SetBackgroundColour("Orange")
            self._power_board_text.SetValue("Standby")
        elif value == "On":
            self._power_board_text.SetBackgroundColour("Light Green")
            self._power_board_text.SetValue("Running")
        else:
            self._power_board_text.SetBackgroundColour("Red")
            self._power_board_text.SetValue("Disabled")

        if not estop:
            self._estop_status.SetBackgroundColour("Red")
            self._estop_status.SetValue("E-Stop Hit")
        else:
            self._estop_status.SetBackgroundColour("Light Green")
            self._estop_status.SetValue("E-Stop OK")
            

    ##\brief Updates test status bar
    def _update_status_bar(self, level, msg):        
        if not self.launched:
            self._status_bar.SetValue("Launch to display status")
            self._status_bar.SetBackgroundColour("White")
        elif level == 0:
            self._status_bar.SetValue("Test Running: OK")
            self._status_bar.SetBackgroundColour("Light Green")
        elif level == 1:
            self._status_bar.SetValue("Warning: %s" % msg)
            self._status_bar.SetBackgroundColour("Orange")
        elif level == 2:
            self._status_bar.SetValue("Error: %s" % msg)
            self._status_bar.SetBackgroundColour("Red")
        elif level == 3:
            self._status_bar.SetValue("Stale Reported: %s" % msg)
            self._status_bar.SetBackgroundColour("Light Blue")
        else:
            self._status_bar.SetBackgroundColour("Light Blue")
            self._status_bar.SetValue("Test Monitor: No Updates/Stale")        

    ##\brief Called after status message or timer callback
    ##\todo Make private
    def update_controls(self, level = 4, msg = 'None'):
        self._update_status_bar(level, msg)

        # These are updated here instead of the callback, because of the stale timer
        self._stat_level = level
        self._test_msg = msg

        remaining = self.calc_remaining()
        remain_str = "N/A" 
        if remaining < 10**6:
            remain_str = get_duration_str(remaining)
        self._done_time_ctrl.SetValue(remain_str)

        self._active_time_ctrl.SetValue(self._record.get_active_str())
        self._elapsed_time_ctrl.SetValue(self._record.get_elapsed_str())

    ##\brief Set pause/reset and power buttons on or off
    def _enable_controls(self):
        self._reset_button.Enable(self.launched)
        self._halt_button.Enable(self.launched)

        self._test_bay_ctrl.Enable(not self.launched)
        self._close_button.Enable(not self.launched)

        # Power buttons
        self._power_run_button.Enable(self.launched and self._bay.board is not None)
        self._power_standby_button.Enable(self.launched and self._bay.board is not None)
        self._power_disable_button.Enable(self.launched and self._bay.board is not None)        

        if self.launched:
            self._launch_button.SetLabel("Stop")
        else:
            self._launch_button.SetLabel("Launch")
        
    def stop_if_done(self):
        remain = self.calc_remaining()
        
        if remain < 0:
            self._stop_count += 1
        else:
            self._stop_count = 0

        # Make sure we've had five consecutive seconds of 
        # negative time before we shutdown
        if self._stop_count > 5 and not self._record.test_complete:
            
            self._record.complete_test()
            self.stop_test()
            self._enable_controls()
        

    ##\todo Private
    def status_callback(self, msg):
        with self._mutex:
            self._status_msg = msg

        wx.CallAfter(self.new_msg)

    ##\todo Private
    def new_msg(self):
        with self._mutex:
            level_dict = { 0: 'OK', 1: 'Warn', 2: 'Error', 3: 'Stale' }

            test_level = self._status_msg.test_ok
            test_msg = self._status_msg.message

        self._last_message_time = rospy.get_time()            

        self._is_running = (self._status_msg.test_ok == 0)
        self._is_stale = False

        self.start_timer()

        self.update_controls(test_level, test_msg)
        self.update_test_record()
        self.stop_if_done()

    ##\todo Private, and move out of class
    def make_launch_script(self, bay, script, local_diag_topic):
        # Set ROS_NAMESPACE ...
        os.environ['ROS_NAMESPACE'] = bay.name

        launch = '<launch>\n'
        launch += '<group ns="%s" >\n' % bay.name

        launch += '<param name="tf_prefix" type="string" value="%s" />\n' % bay.name
        # Remap
        launch += '<remap from="/diagnostics" to="%s" />\n' % local_diag_topic
        
        # Init machine
        # Root on remote 
        launch += '<machine name="test_host_root" user="root" address="%s" ' % bay.machine
        launch += 'ros-root="$(env ROS_ROOT)" ros-package-path="$(env ROS_PACKAGE_PATH)" timeout="15" default="never"/>\n'

        # Set default to remote machine
        launch += '<machine name="test_host" address="%s" default="true" ' % bay.machine
        launch += 'ros-root="$(env ROS_ROOT)" ros-package-path="$(env ROS_PACKAGE_PATH)" timeout="15"  />\n'
        
        # Local host
        launch += '<machine name="localhost" address="localhost" '
        launch += 'ros-root="$(env ROS_ROOT)" ros-package-path="$(env ROS_PACKAGE_PATH)" timeout="20" default="false"/>\n'

        # Include our launch file
        launch += '<include file="$(find life_test)/%s" />\n' % script

        # Rosbag launches and records local diagnostics
        launch += ' <node machine="localhost" pkg="rosbag" type="rosbag" '
        launch += 'args="record -o /hwlog/%s_life_test /diagnostics --split 1000" name="test_logger" />\n' % self._serial

        # Rosrecord launches - will record burst of data on trigger
        launch += ' <node machine="localhost" pkg="rosrecord" type="rosrecord" name="snapshot_record" '
        launch += 'args="-f /hwlog/%s_test_events joint_states mechanism_statistics -s " />\n' % self._serial

        # Rosbag records our motor traces
        launch += ' <node machine="localhost" pkg="rosbag" type="rosbag" name="mtrace_record" '
        launch += 'args="record -o /hwlog/%s_motor_trace -e %s/motor_trace/.* " />\n' % (self._serial, bay.name)
        
        launch += '</group>\n</launch>'

        return launch


    # Put in master file
    # Add subscriber to diagnostics
    # Launch file, subscribe diagnostics
    def start_stop_test(self, event):
        if self.launched:
            if not self.stop_test_user():
                return
        else:
            if not self.launch_test():
                return
            

        self.update_controls()
        self._enable_controls()

    ##\brief Called when user presses "stop" button
    def stop_test_user(self):
        dialog = wx.MessageDialog(self, 'Are you sure you want to stop this test?', 'Confirm Stop Test', wx.OK|wx.CANCEL)
        if dialog.ShowModal() != wx.ID_OK:
            return False

        self.stop_test()
        return True

    def _tear_down_test(self):
        """
        Reset displays and state. Shut down test processes
        """
        # Lockout GUI
        self._launch_button.Enable(False)

        # Stop the test
        self.on_halt_test(None)

        # Update displays
        self._power_board_text.SetBackgroundColour("White")
        self._power_board_text.SetValue("Test Not Running")
        self._estop_status.SetBackgroundColour("White")
        self._estop_status.SetValue("Test Not Running")
        self._machine_text.SetValue("Not running")
        self._power_sn_text.SetValue("Not running")
        self._power_breaker_text.SetValue("Not running")

        # Kill the power
        if self._bay.board is not None:
            if not self._manager.power_disable(self._bay):
                wx.MessageBox("Power disable command failed. Unable to command power board", "Power command failed", wx.OK|wx.ICON_ERROR, self)

        
        # Shutdown processes
        if self._test_launcher:
            self._test_launcher.shutdown()
        self._manager.test_stop(self._bay)
        self.update_test_record('Shutting down test processes.')

        # Reset state
        self._is_running = False
        self._test_launcher = None  
        self._bay = None
        self._record.set_bay(None)

        # Enable GUI
        self._launch_button.Enable(True)
        self._launch_button.SetLabel("Launch")
        self.update_controls()
        self._enable_controls()

    def stop_test(self):
        if self.launched:
            self._tear_down_test()

        if self._status_sub:
            self._status_sub.unregister()
        self._status_sub = None

        self._is_running = False

    def _check_machine(self, bay):
        """
        @brief Check the machine is online.

        @return True if machine is OK
        """
        try:
            machine_addr = socket.gethostbyname(bay.machine)
        except socket.gaierror:
            wx.MessageBox('Hostname "%s" (bay "%s") is invalid. The machine may be offline or disconnected.' % (bay.machine, bay.name),
                          'Test Bay Invalid', wx.OK)
            return False

        # Check that it is pingable
        retcode = subprocess.call('ping -c1 -W1 %s > /dev/null' % bay.machine, shell=True)
        if retcode != 0:
            wx.MessageBox('Cannot contact machine "%s" for bay "%s". It may be offline or disconnected. Check the machine and retry.' % (bay.machine, bay.name),
                          'Test Bay Unavailable', wx.OK)
            return False

        return True

    def _load_bay(self):
        """
        @brief Checks that the bay is valid, reserves bay, runs power.

        @return None if bay invalid
        """
        bay_name = self._test_bay_ctrl.GetStringSelection()
        bay = self._manager.room.get_bay(bay_name)
        if bay is None:
            wx.MessageBox('Select test bay', 'Select Bay', wx.OK|wx.ICON_ERROR, self)
            return None

        return bay


    def _enable_bay(self):
        """
        @brief Reserve bay from manager, enable power
        
        @return False if unable to reserve bay or enable power
        """
        if not self._manager.test_start_check(self._bay, self._serial):
            wx.MessageBox('Test bay in use, select again!', 'Test bay in use', wx.OK|wx.ICON_ERROR, self)
            return False

        if self._bay.board is not None:
            if not self._manager.power_run(self._bay):
                wx.MessageBox('Unable to command power board. Check connections. Board: %s, Breaker %d' % (self._bay.board, self._bay.breaker), 'Power Command Failure', wx.OK|wx.ICON_ERROR, self)
                self._manager.test_stop(self._bay)
                return False

        return True

    def _check_test_ready(self):
        """
        @brief Check if test has <0 time remaining.

        @return False if test has no time left
        """
        if self.calc_remaining() <= 0:
            wx.MessageBox('Test has no allowable time left. Add more hours/minutes and retry.',
                          'Out of Time', wx.OK|wx.ICON_ERROR, self)
            return False
        return True

    def _confirm_launch(self):
        """
        @brief Checks with user to make sure test can start.

        @return True if user is OK
        """
        dialog = wx.MessageDialog(self, 'Are you sure you want to launch?', 'Confirm Launch', wx.OK|wx.CANCEL)
        ok = dialog.ShowModal() == wx.ID_OK
        dialog.Destroy()

        return ok
        
    def launch_test(self):
        """
        @brief Launches test on correct bay

        @return False if launch failed or aborted
        """
        # Lock out launch button
        self._launch_button.Enable(False)
        self._test_complete = False

        # Check to make sure we have time
        if not self._check_test_ready():
            self._launch_button.Enable(True)
            return False

        # Load the bay from the selection bar
        bay = self._load_bay()
        if not bay:
            self._launch_button.Enable(True)
            return False

        # Make sure the machine is OK
        if not self._check_machine(bay):
            self._launch_button.Enable(True)
            return False

        # Confirm with user
        if not self._confirm_launch():
            self._launch_button.Enable(True)
            return False

        self._bay = bay
        self._record.set_bay(self._bay)

        # Reserve the bay, enable power
        if not self._enable_bay():
            self._launch_button.Enable(True)
            self._bay = None
            self._record.set_bay(None)
            return False
        
        # Set GUI for running test
        self._machine_text.SetValue(self._bay.machine)
        if self._bay.board is not None and self._test.needs_power:
            self._power_sn_text.SetValue(str(self._bay.board))
            self._power_breaker_text.SetValue(str(self._bay.breaker))
            self._power_board_text.SetValue("No data")
            self._power_board_text.SetBackgroundColour("Light Blue")
            self._estop_status.SetValue("No data")
            self._estop_status.SetBackgroundColour("Light Blue")
            self._power_stat = "No data"
            self._estop_stat = False
        else:
            self._power_sn_text.SetValue("No board")
            self._power_breaker_text.SetValue("No breaker")
            self._power_board_text.SetValue("No board")
            self._power_board_text.SetBackgroundColour("White")
            self._estop_status.SetBackgroundColour("White")
            self._estop_status.SetValue("No board")

        self.update_test_record('Launching test %s on bay %s, machine %s.' % (self._test._name, self._bay.name, self._bay.machine))

        # Local diagnostic topic
        local_diag = '/' + self._bay.name + '/diagnostics'

        # Clear namespace in bay
        rospy.set_param(self._bay.name, {})
        self._test.set_params(self._bay.name)
        self._test_launcher = roslaunch_caller.ScriptRoslaunch(
            self.make_launch_script(self._bay, self._test.launch_file, local_diag))
        try:
            self._test_launcher.start()
        except Exception, e:
            traceback.print_exc()
            self.update_test_record('Failed to launch script, caught exception.')
            self.update_test_record(traceback.format_exc())
            self._tear_down_test()
            self._launch_button.Enable(True)            

            wx.MessageBox('Failed to launch. Check machine for connectivity. See log for error message.', 'Failure to launch', wx.OK|wx.ICON_ERROR, self)
            return False

        # Test successfully launched
        local_status = '/' + str(self._bay.name) + '/test_status'
        self._is_running = False # We're not running until we hear back from test
        self._monitor_panel.change_diagnostic_topic(local_diag)
        self._status_sub = rospy.Subscriber(local_status, TestStatus, self.status_callback)

        self.update_controls()
        self._enable_controls()
        self._launch_button.Enable(True)

        return True
        
    def on_halt_test(self, event = None):
        """
        Calls halt_test service to test monitor
        """
        try:
            self.update_test_record('Pausing test.')
            halt_srv = rospy.ServiceProxy(self._bay.name + '/halt_test', Empty)
            halt_srv()

        except Exception, e:
            rospy.logerr('Exception on halt test.\n%s' % traceback.format_exc())

    def on_reset_test(self, event = None):
        """
        Calls reset_test service to test monitor
        """
        try:
            self.update_test_record('Resetting test.')
            reset = rospy.ServiceProxy(self._bay.name + '/reset_test', Empty)
            reset()
            
        except:
            rospy.logerr('Exception on reset test.\n%s' % traceback.format_exc())
      

    def record_test_log(self):
        """
        Called when test is closing down
        """
        self._record.load_attachments(self._manager.invent_client)

    def on_invent_timer(self, event):
        """
        Update inventory system with a note on progress every 10 minutes
        """
        self._record.update_invent(self._manager.invent_client)

        
        
