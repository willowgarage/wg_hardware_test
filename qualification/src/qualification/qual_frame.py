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
##\brief Contains main functionality for WG production qualification system. 

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

from pr2_self_test_msgs.srv import ScriptDone, ScriptDoneRequest, ScriptDoneResponse
from pr2_self_test_msgs.srv import TestResult, TestResultRequest, TestResultResponse

from qualification.test import *
from qualification.result import *

import traceback

from wg_invent_client import Invent
from runtime_monitor.monitor_panel import MonitorPanel
from roslaunch_caller import roslaunch_caller 

import rxtools.cppwidgets

##\brief Passed to qualification manager
##
## Holds data about item to be qualified. Subtests can hold more data
class QualTestObject(object):
  def __init__(self, name, serial):
    self.name = name
    self.serial = serial

class QualOptions(object):
  def __init__(self):
    self.debug = False
    self.always_show_results = False

##\brief Loads instructions for operator. Displays instructions in HTML window.
class InstructionsPanel(wx.Panel):
  def __init__(self, parent, resource, qualification_frame, file):
    wx.Panel.__init__(self, parent)
    
    self._manager = qualification_frame
    
    self._panel = resource.LoadPanel(self, 'instructions_panel')
    self._sizer = wx.BoxSizer(wx.HORIZONTAL)
    
    self._html_window = xrc.XRCCTRL(self._panel, 'html_window')
    self._continue_button = xrc.XRCCTRL(self._panel, 'continue_button')
    self._html_window.LoadFile(file)
    
    self._continue_button.Bind(wx.EVT_BUTTON, self.on_continue)
    self._continue_button.SetFocus()
    
    self._sizer.Add(self._panel, 1, wx.EXPAND|wx.ALIGN_CENTER_VERTICAL)
    self.SetSizer(self._sizer)
    self.Layout()
    self.Fit()
    
    self._cancel_button = xrc.XRCCTRL(self._panel, 'cancel_button')
    self._cancel_button.Bind(wx.EVT_BUTTON, self.on_cancel)
    
  def on_continue(self, event):
    self._manager.start_qualification()
    
  def on_cancel(self, event):
    # We reset here and don't cancel, because nothing is running
    self._manager.reset()
  
##\brief Displays waiting page, subtest results
## 
## Plots panel displays a waiting page while waiting for subtests to 
## complete, displays subtest results, displays runtime monitor 
class PlotsPanel(wx.Panel):
  def __init__(self, parent, resource, qualification_frame):
    wx.Panel.__init__(self, parent)
    
    self._manager = qualification_frame
    
    self._panel = resource.LoadPanel(self, 'plots_panel')
    self._sizer = wx.BoxSizer(wx.HORIZONTAL)
    
    self._sizer.Add(self._panel, 1, wx.EXPAND)
    self.SetSizer(self._sizer)
    self.Layout()
    
    self._plots_window = xrc.XRCCTRL(self._panel, 'plots_window')
    self._pass_button = xrc.XRCCTRL(self._panel, 'pass_button')
    self._fail_button = xrc.XRCCTRL(self._panel, 'fail_button')
    self._retry_button = xrc.XRCCTRL(self._panel, 'retry_button')
    
    self._pass_button.Bind(wx.EVT_BUTTON, self.on_pass)
    self._fail_button.Bind(wx.EVT_BUTTON, self.on_fail)
    self._retry_button.Bind(wx.EVT_BUTTON, self.on_retry)

    self._notes_text = xrc.XRCCTRL(self._panel, 'notes_text')
     
    self._cancel_button = xrc.XRCCTRL(self._panel, 'cancel_button')
    self._cancel_button.Bind(wx.EVT_BUTTON, self.on_cancel)

    # Make runtime monitor panel, console panel
    self._notebook = xrc.XRCCTRL(self._panel, 'results_notebook')
    self._create_monitor()
    self._create_rxconsole()

  def _create_monitor(self):
    self._monitor_panel = MonitorPanel(self._notebook)
    self._monitor_panel.SetSize(wx.Size(400, 500))
    self._notebook.AddPage(self._monitor_panel, "Diagnostics")

  def _create_rxconsole(self):
    rxtools.cppwidgets.initRoscpp("qualification_console", True)

    self._rxconsole_panel = rxtools.cppwidgets.RosoutPanel(self._notebook)
    self._rxconsole_panel.SetSize(wx.Size(400, 500))
    self._rxconsole_panel.setEnabled(True)
    self._notebook.AddPage(self._rxconsole_panel, "ROS Output")
    
  def close(self):
    self._monitor_panel.shutdown()
    # Maybe do something with rxconsole to shut down

  ##\brief Displays plots, HTML of results
  ##
  ##\param result_page str : HTML result page of results
  ##\param pass_ok bool : Pass button only enabled when true
  def show_plots(self, result_page, pass_ok):
    self._notes_text.SetEditable(True)
    self._notebook.SetSelection(0)
    self._plots_window.SetPage(result_page)
    
    # Pass is only enabled when in debug mode or test has passed
    self._pass_button.Enable(pass_ok)

    self._fail_button.Enable(True)
    self._retry_button.Enable(True)
    self._cancel_button.Enable(True)
    self._pass_button.SetFocus()
      
  def show_waiting(self, wait_page, cancel_enabled = True):
    self._notes_text.SetEditable(False)
    self._plots_window.SetPage(wait_page)
    self._pass_button.Enable(False)
    self._fail_button.Enable(False)
    self._retry_button.Enable(False)
    self._cancel_button.Enable(cancel_enabled)
    
  def on_pass(self, event):
    notes =  self._notes_text.GetValue()
    self._notes_text.Clear()
    self._manager.subtest_result(True, notes)
  
  def on_fail(self, event):
    notes = self._notes_text.GetValue()
    self._notes_text.Clear()
    self._manager.subtest_result(False, notes)

  def on_retry(self, event):
    notes = self._notes_text.GetValue()
    self._manager.retry_subtest(notes)
    self._notes_text.Clear()
    
  def on_cancel(self, event):
    self._manager.cancel()
    
##\brief Displays results of entire qualification tests
##
## Displays results of qualification tests. Results come in as a QualTestResult 
## class, which makes an HTML page. Users must submit to inventory system.
class ResultsPanel(wx.Panel):
  def __init__(self, parent, resource, qualification_frame):
    wx.Panel.__init__(self, parent)
    
    self._manager = qualification_frame
    
    self._panel = resource.LoadPanel(self, 'results_panel')
    self._sizer = wx.BoxSizer(wx.HORIZONTAL)
    
    self._sizer.Add(self._panel, 1, wx.EXPAND|wx.ALIGN_CENTER_VERTICAL)
    self.SetSizer(self._sizer)
    self.Layout()
    self.Fit()
    
    self._submit_button = xrc.XRCCTRL(self._panel, 'submit_button')
    self._results_window = xrc.XRCCTRL(self._panel, 'results_window')

    self._notesbox = xrc.XRCCTRL(self._panel, 'notes_text')
    self._submit_button.Bind(wx.EVT_BUTTON, self.on_submit)
    self._submit_button.SetFocus()

    self._dir_picker = xrc.XRCCTRL(self._panel, 'results_dir_picker')
    
  def set_results(self, results):
    self._results_window.Freeze()
    self._results_window.SetPage(results.make_summary_page())
    self._results_window.Thaw()
    self._dir_picker.SetPath(results._results_dir)

  def on_submit(self, event):
    self._manager.submit_results(self._notesbox.GetValue(), self._dir_picker.GetPath())

##\brief Prestartup or shutdown scripts, returns timeout msg
def script_timeout(timeout):
  if timeout > 0:
    msg = 'Automated timeout. Timeout: %s' % timeout
  else:
    msg = 'Subtest aborted'

  return ScriptDoneRequest(1, msg)

##\brief Subtest timeout, returns timeout message
def subtest_timeout(timeout):
  if timeout > 0:
    msg = 'Automated timeout. Timeout: %s' % timeout
  else:
    msg = 'Subtest aborted'

  result = TestResultRequest()
  result.html_result = '<p>%s</p>\n' % msg
  result.text_summary = msg
  result.plots = []
  result.result = TestResultRequest.RESULT_FAIL 

  return result
  
##\brief Makes waiting page for subtests
def make_html_waiting_page(subtest_name, index, len_subtests):
  html = ['<html><H2 align=center>Waiting for Subtest to Complete</H2>']
  html.append('<H3 align=center>Test Name: %s</H3>' % subtest_name)
  html.append('<H4 align=center>Test %d of %d</H4></html>' % (index + 1, len_subtests))

  return '\n'.join(html)

##\brief Main frame of qualification
##
## Loads tests, launches them and records results
class QualificationFrame(wx.Frame):
  def __init__(self, parent, options):
    wx.Frame.__init__(self, parent, wx.ID_ANY, "Qualification")

    self.options = options

    self._result_service = None
    self._prestartup_done_srv = None
    self._shutdown_done_srv = None

    self._invent_client = None
        
    # Load the XRC resource
    xrc_path = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'xrc/gui.xrc')
    self._res = xrc.XmlResource(xrc_path)

    # Load the main panel
    self._root_panel = self._res.LoadPanel(self, 'main_panel')

    self._top_panel = xrc.XRCCTRL(self._root_panel, "top_panel")
    self._top_sizer = wx.BoxSizer(wx.HORIZONTAL)
    self._top_panel.SetSizer(self._top_sizer)
    self._current_panel = None
    self._plots_panel = None

    self._log_panel = xrc.XRCCTRL(self._root_panel, "log_panel")
    self._log = xrc.XRCCTRL(self._log_panel, 'log')

    self._results = None
    
    self._startup_launch = None
    self._shutdown_launch = None
    self._subtest_launch = None
    self._prestartup_launch = None
    self._record_launch = None

    # Services advertised for prestarts, subtests and shutdown scripts
    self._prestartup_done_srv = None
    self._result_service = None
    self._shutdown_done_srv = None

    # Timers make sure we can timeout safely
    self._prestart_timer = None
    self._subtest_timer = None
    self._shutdown_timer = None

    self._current_test = None
 
    rospy.set_param('/invent/username', '')
    rospy.set_param('/invent/password', '')

    self._waiting_for_submit = False

    self.Bind(wx.EVT_CLOSE, self.on_close)

    self._spin_timer = wx.Timer(self, wx.ID_ANY)
    self.Bind(wx.EVT_TIMER, self.on_spin, self._spin_timer)
    self._spin_timer.Start(100)

    self.log("Startup")
    self.reset()

  ##\brief Logs test results in test log, displays to user
  def log(self, msg):
    log_msg = datetime.now().strftime("%m/%d/%Y %H:%M:%S: ") + msg
    if self._results:
      self._results.log(msg)
    self._log.AppendText(log_msg + '\n')
    self._log.Refresh()
    self._log.Update()
    
  ##\brief Sets top panel of main panel
  def set_top_panel(self, panel):
    if self._current_panel == panel:
      return

    self._current_panel = panel
    self._top_sizer.Clear(True)
    self._top_sizer.Add(self._current_panel, 1, wx.EXPAND|wx.ALIGN_CENTER_VERTICAL)
    self._top_panel.Layout()
    
  ##\brief Resets the frame to the starting state. 
  def reset(self):
    # get_loader_panel overridden by subclasses
    loader_panel = self.get_loader_panel()
    self.set_top_panel(loader_panel)
    if self._results:
      self._results.close()
    self._results = None

    if self._plots_panel:
      self._plots_panel.close()
    self._plots_panel = None

    self.reset_params()
  
  ##\brief Resets parameters of qualification node to starting state
  def reset_params(self):
    run_id = rospy.get_param('/run_id')
    roslaunch_params = rospy.get_param('/roslaunch', '')
    qual_params = rospy.get_param('/qualification', '')
    invent_params = rospy.get_param('/invent', '')
    # Reset all parameters
    rospy.set_param('/', {'run_id': run_id,
                          'roslaunch' : roslaunch_params,
                          'qualification' : qual_params,
                          'invent' : invent_params})

  ##\brief Called from loader panel
  ##
  ##\todo Rename 'begin_test' to something useful at some point
  def begin_test(self, test, qual_item):
    ##\todo If debug mode, allow proceed with message dialog
    if not self.get_inventory_object():
      wx.MessageBox('Cannot proceed without inventory login. You must have an inventory username and password.', 'Inventory Login Required', wx.OK|wx.ICON_ERROR, self)
      self.reset()
      return    

    self._current_test = test
    self._current_item = qual_item

    rospy.set_param('/qual_item/serial', self._current_item.serial)
    rospy.set_param('/qual_item/name', self._current_item.name)
    
    if (self._current_test.getInstructionsFile() != None):
      self.set_top_panel(InstructionsPanel(self._top_panel, self._res, self, self._current_test.getInstructionsFile()))
    else:
      self.start_qualification()
   
  ##\brief Launches program for either onboard, component conf or test cart tests
  def start_qualification(self):
    if (len(self._current_test.subtests) == 0):
      wx.MessageBox('Test selected has no subtests defined', 'No tests', wx.OK|wx.ICON_ERROR, self)
      return

    self.record_tests()

    self._tests_start_date = datetime.now()
   
    self._results = QualTestResult(self._current_item, self._current_test, self._tests_start_date)

    # Create plots panel to show plots, runtime monitor.
    self._plots_panel = PlotsPanel(self._top_panel, self._res, self)

    self.run_prestartup_scripts()
    
  ##\brief Launches rosrecord node to record diagnostics for test
  def record_tests(self):
    ##\todo Use file in qual package as recorder
    record = '<launch>\n'
    record += '<node pkg="rosrecord" type="rosrecord" name="test_logger" \n'
    record += 'args="-f /hwlog/%s_qual_test /diagnostics" />\n' % self._current_item.serial
    record += '</launch>\n'

    self._record_launch = self.launch_script(record)
    if (self._record_launch == None):
      rospy.logerr('Couldn\'t launch rosrecord for qualification test')
      return

  ##\brief Run any pre_startup scripts synchronously
  def run_prestartup_scripts(self):
    if (len(self._current_test.pre_startup_scripts) == 0):
      self.log('No prestartup scripts.')
      self.test_startup()
      return

    self._prestartup_index = 0

    if (self._prestartup_done_srv != None):
      self._prestartup_done_srv.shutdown()
      self._prestartup_done_srv = None

    self.log('Running prestartup scripts')

    self.prestartup_call()

  ##\brief Launches prestartup script
  def prestartup_call(self):
    prestart = self._current_test.pre_startup_scripts[self._prestartup_index]
      
    script = prestart.launch_file
    name = prestart.get_name()

    # update script label in waiting panel
    log_message = 'Running pre_startup script [%s]...'%(name)
    self.log(log_message)

    wait_html = '<html><H2 align=center>Prestartup Scripts Running</H2>\n'
    wait_html += '<H3 align=center>Script: %s</H3>\n</html>' % name

    self._plots_panel.show_waiting(wait_html)

    self.set_top_panel(self._plots_panel)

    self._prestartup_done_srv = rospy.Service('prestartup_done', ScriptDone, self.prestartup_done_callback)

    self._prestartup_launch = self.launch_file(script)
    if (self._prestartup_launch == None):
      s = 'Could not load roslaunch script "%s"! Press OK to cancel test.' % (os.path.basename(script))
      wx.MessageBox(s, 'Invalid roslaunch file', wx.OK|wx.ICON_ERROR, self)
      self.cancel(s)
      return

    # Sets timeout for prestarts
    if prestart.get_timeout() > 0:
      timeout = prestart.get_timeout()
      self._prestart_timer = threading.Timer(timeout,
                                             self.prestartup_done_callback, 
                                             [script_timeout(timeout)])
      self._prestart_timer.start()
                                             
  ##\brief Hit from 'prestartup_done' service or timeout
  def prestartup_done_callback(self, srv):
    wx.CallAfter(self.prestartup_finished, srv)
    return ScriptDoneResponse()

  ## Checks result of last prestartup script. Continues with prestartup or 
  ## runs startup/subtests if OK, fails if error.
  def prestartup_finished(self, srv):
    if self._prestartup_done_srv:
      self._prestartup_done_srv.shutdown()
      self._prestartup_done_srv = None

    if self._prestartup_launch != None:
      self._prestartup_launch.shutdown()
      self._prestartup_launch = None

    if self._prestart_timer:
      self._prestart_timer.cancel()
      self._prestart_timer = None

    result_dict = { 0: 'OK', 1: 'FAIL', 2: 'ERROR' }
    self.log('Prestartup script finished. Result %s.' % (result_dict[srv.result]))

    self._results.add_prestartup_result(self._prestartup_index, srv)
    
    if srv.result == 0:
      # Continue to next test
      self._prestartup_index += 1
      if self._prestartup_index >= len(self._current_test.pre_startup_scripts):
        self.test_startup()
      else:
        self.prestartup_call()
    else: # Failure or error 
      self.test_finished()
      
  ##\brief Launches startup script (if any) and first subtest.
  def test_startup(self):
    # Run the startup script if we have one
    startup = self._current_test.getStartupScript()

    if startup:
      self.log('Running startup script...')
      self._startup_launch = self.launch_file(startup.launch_file)
      if (self._startup_launch == None):
        s = 'Could not load roslaunch script %s, file: "%s"' % (startup.get_name(), os.path.basename(startup.launch_file))
        wx.MessageBox(s, 'Invalid roslaunch file. Press OK to cancel.', wx.OK|wx.ICON_ERROR, self)
        self.cancel(s)
        return
      
      rospy.sleep(2.0)
      rospy.loginfo('Sleeping after startup script')

    else:
      self.log('No startup script, launching subtests')
      
    self.start_subtest(0)

  ##\brief Launches subtest and displays waiting page
  ##@param index int : Index of subtest to launch
  def start_subtest(self, index):
    self._subtest_index = index

    subtest = self._current_test.subtests[self._subtest_index]

    self._plots_panel.show_waiting(make_html_waiting_page(subtest.get_name(), index, len(self._current_test.subtests)))
    self.set_top_panel(self._plots_panel)
    
    self.launch_pre_subtest()

    self._result_service = rospy.Service('test_result', TestResult, self.subtest_callback)

    script = subtest._test_script
    self._subtest_launch = self.launch_file(script)
    if (self._subtest_launch == None):
      s = 'Could not load roslaunch script "%s"'%(os.path.basename(script))
      wx.MessageBox(s, 'Invalid roslaunch file. Press OK to cancel.', wx.OK|wx.ICON_ERROR, self)
      self.cancel(s)
      return

    # Sets timeout for subtests
    if subtest.get_timeout() > 0:
      timeout = subtest.get_timeout()
      self._subtest_timer = threading.Timer(timeout,
                                             self.subtest_callback, 
                                             [ subtest_timeout(timeout) ])
      self._subtest_timer.start()
 
  ##\brief Callback for subtest results or for timeout
  ##@param msg qualification/TestResultRequest : Test result processed by analysis
  def subtest_callback(self, msg):
    wx.CallAfter(self.subtest_finished, msg)
    return TestResultResponse()   
    
  ## Adds subtest results to logs, displays results if needed
  ## If pass, starts next script. If fail, shuts down
  ##@param msg qualification/TestResultRequest : Test result processed by analysis
  def subtest_finished(self, msg):
    self._subtest_launch.shutdown()
    self._subtest_launch = None

    # Kill subtest service here, too
    if self._result_service:
      self._result_service.shutdown()
      self._result_service = None

    if self._subtest_timer:
      self._subtest_timer.cancel()
      self._subtest_timer = None
    
    sub_result = self._results.add_sub_result(self._subtest_index, msg)

    if self.options.always_show_results:
      self.show_plots(sub_result)
    else:
      if (msg.result == TestResultRequest.RESULT_PASS):
        self.subtest_result(True, '')
      elif (msg.result == TestResultRequest.RESULT_FAIL):
        self.subtest_result(False, '')
      elif (msg.result == TestResultRequest.RESULT_HUMAN_REQUIRED):
        self.log('Subtest "%s" needs human response.'%(self._current_test.subtests[self._subtest_index].get_name()))
        self.show_plots(sub_result)

  ##\brief Displays results of qualification subtests 
  def show_plots(self, sub_result):
    self._plots_panel.show_plots(sub_result.make_result_page(), 
                                 sub_result.get_pass_bool() or self.options.debug)
    self.set_top_panel(self._plots_panel)

  ##\brief Records final result of subtest. 
  ##@param pass_bool bool : Operator passed or failed subtest
  ##@param operator_notes str : Notes operator gave about subtest
  def subtest_result(self, pass_bool, operator_notes):
    self.log('Subtest "%s" result: %s'%(self._current_test.subtests[self._subtest_index].get_name(), pass_bool))
    
    sub_result = self._results.get_subresult(self._subtest_index)
    sub_result.set_note(operator_notes)
    sub_result.set_operator_result(pass_bool)

    if pass_bool:
      self.launch_post_subtest()
      self.next_subtest()
    else:
      self.test_finished() # Terminate rest of test after failure

  ##\brief Retries subtest, logs retry results. 
  def retry_subtest(self, notes):
    self.log('Retrying subtest "%s"'%(self._current_test.subtests[self._subtest_index].get_name()))
    self._results.retry_subresult(self._subtest_index, notes)
    self.start_subtest(self._subtest_index)

  ##\brief Proceed to next subtest if we have one, or finish test.
  def next_subtest(self):
    if (self._subtest_index + 1 >= len(self._current_test.subtests)):
      self.test_finished()
    else:
      self.start_subtest(self._subtest_index + 1)
    
  ##\brief Launches pre subtest scripts if any, blocks until complete.
  def launch_pre_subtest(self):
    subtest = self._current_test.subtests[self._subtest_index]

    if (subtest._pre_script is not None):
      script = subtest._pre_script
      pre_launcher = self.launch_file(script)
      if (pre_launcher == None):
        s = 'Could not load pre-subtest roslaunch script "%s"'%(os.path.basename(script))
        wx.MessageBox(s, 'Invalid roslaunch file', wx.OK|wx.ICON_ERROR, self)
        self.cancel(s)
      else:
        pre_launcher.spin()      

  ##\brief Launches post subtest scripts if any, blocks until complete.
  def launch_post_subtest(self):
    subtest = self._current_test.subtests[self._subtest_index]

    if (subtest._post_script is not None):
      script = subtest._post_script
      post_launcher = self.launch_file(script)
      if (post_launcher == None):
        s = 'Could not load post-subtest roslaunch script "%s"'%(os.path.basename(script))
        wx.MessageBox(s, 'Invalid roslaunch file', wx.OK|wx.ICON_ERROR, self)
        self.cancel(s)
      else:
        post_launcher.spin() 
  
  ##\todo Move out of class, don't need class variables
  ##\brief Uses roslaunch_caller to launch file
  ##@param file str : Full path of launch script
  def launch_file(self, file):
    f = open(file, 'r')
    launch_xml = f.read()
    f.close()
    
    return self.launch_script(launch_xml)

  ##\brief Uses roslaunch_caller to launch script
  ##@param script str : XML string to launch
  def launch_script(self, script):
    launch = roslaunch_caller.ScriptRoslaunch(script, None)
    try:
      launch.start()
    except Exception, e:
      traceback.print_exc()
      self.log('Caught exception launching file:\n%s' % traceback.format_exc())
      return None
  
    return launch
    
  ##\brief Spins all launch files once, called by timer
  def on_spin(self, event):
    if (self._subtest_launch != None):
      self._subtest_launch.spin_once()
    
    if (self._startup_launch != None):
      self._startup_launch.spin_once()
      
    if (self._shutdown_launch != None):
      self._shutdown_launch.spin_once()
      
    if (self._prestartup_launch != None):
      self._prestartup_launch.spin_once()

    if (self._record_launch != None):
      self._record_launch.spin_once()
         
  ##\brief Checks if we have any launch files running
  ##
  ##\return True if any launch files running
  def launches_running(self):
    if self._subtest_launch is not None: return True
    if self._startup_launch is not None: return True
    if self._shutdown_launch is not None: return True
    if self._prestartup_launch is not None: return True
    if self._record_launch is not None: return True

    return False

  ##\brief Stops all running launch files, blocks until complete
  def stop_launches(self):
    self.log('Stopping launches')
    
    # Launches
    if self._subtest_launch is not None:
      self._subtest_launch.shutdown()
    
    if self._startup_launch is not None:
      self._startup_launch.shutdown()
      
    if self._shutdown_launch is not None:
      self._shutdown_launch.shutdown()

    if self._prestartup_launch is not None:
      self._prestartup_launch.shutdown()

    if self._record_launch is not None:
      self._record_launch.shutdown()
      
    # Services advertised
    if self._shutdown_done_srv is not None:
      self._shutdown_done_srv.shutdown()

    if self._result_service is not None:
      self._result_service.shutdown()
        
    if self._prestartup_done_srv is not None:
      self._prestartup_done_srv.shutdown()

    # Timers
    if self._subtest_timer is not None:
      self._subtest_timer.cancel()
    
    if self._prestart_timer is not None:
      self._prestart_timer.cancel()
    
    if self._shutdown_timer is not None:
      self._shutdown_timer.cancel()

    self._subtest_timer = None
    self._prestart_timer = None
    self._shutdown_timer = None

    self._result_service = None
    self._shutdown_done_srv = None
    self._prestartup_done_srv = None
      
    self._startup_launch = None
    self._shutdown_launch = None
    self._prestartup_launch = None
    self._subtest_launch = None
    self._record_launch = None

    self.log('Launches stopped.')

  ##\brief Launches shutdown script if we have one
  def test_finished(self):
    if (self._current_test is not None and self._current_test.getShutdownScript() != None):
      self.log('Running shutdown script...')
      
      if (self._shutdown_done_srv != None):
        self._shutdown_done_srv.shutdown()
        self._shutdown_done_srv = None
      
      self._shutdown_done_srv = rospy.Service('shutdown_done', ScriptDone, self.shutdown_callback)

      html = '<html><H2 align=center>Shutting down test and killing processes</H2></html>'
      
      self._plots_panel.show_waiting(html, False)
      self.set_top_panel(self._plots_panel)

      # Launch given shutdown script
      shutdown_script = self._current_test.getShutdownScript()
      script = shutdown_script.launch_file
      
      self._shutdown_launch = self.launch_file(script)
          
      if self._shutdown_launch == None:
        s = 'Could not load roslaunch shutdown script "%s". SHUTDOWN POWER BOARD MANUALLY!'%(os.path.basename(script))
        wx.MessageBox(s, 'Invalid shutdown script!', wx.OK|wx.ICON_ERROR, self)
        self.log('No shutdown script: %s' % (s))
        self.log('SHUT DOWN POWER BOARD MANUALLY')
        return

      if shutdown_script.get_timeout() > 0:
          timeout = shutdown_script.get_timeout()
          self._shutdown_timer = threading.Timer(timeout,
                                                 self.shutdown_callback,
                                                 [ script_timeout(timeout) ])
          self._shutdown_timer.start()
    else:
      self.log('No shutdown script')
      self.test_cleanup()

  ##\brief Callback for shutdown script
  ##@param srv qualification/ScriptDoneRequest : Result of shutdown script
  def shutdown_callback(self, srv):
    wx.CallAfter(self.shutdown_finished, srv)
    return ScriptDoneResponse()

  ##\brief Stops shutdown launch, cleans up test
  def shutdown_finished(self, srv):
    self.log('Shutdown finished')

    if self._current_test is None:
      self.log('No test, how can I be shutting down?')
      return

    if self._shutdown_launch is not None:
      self._shutdown_launch.shutdown()
      self._shutdown_launch = None

    if self._shutdown_done_srv is not None:
      self._shutdown_done_srv.shutdown()
      self._shutdown_done_srv = None

    if self._shutdown_timer is not None:
      self._shutdown_timer.cancel()
      self._shutdown_timer = None

    self._results.add_shutdown_result(srv)

    if srv.result != ScriptDoneRequest.RESULT_OK:
      fail_msg = 'Shutdown script failed!'
      wx.MessageBox(fail_msg + '\n' + srv.failure_msg, fail_msg, wx.OK|wx.ICON_ERROR, self)
      self.log('Shutdown failed for: %s' % srv.failure_msg)

    self.test_cleanup()

  ##\brief Stops all launches, shows results when done 
  def test_cleanup(self):
    self.stop_launches()
    
    # Should this be in self.reset?
    self._current_test = None
    self._subtest_index = 0

    self.show_results()

  ##\brief Shows final results of qualification test
  def show_results(self):
    panel = ResultsPanel(self._top_panel, self._res, self)
    self.set_top_panel(panel)
    
    self._waiting_for_submit = True

    if self._results is not None:
      self._results.write_results_to_file() # Write to temp dir
      panel.set_results(self._results)
    else:
      self.reset()

  ##\brief Gets invent login from username/password
  def login_to_invent(self):
    dialog = self._res.LoadDialog(self, 'username_password_dialog')
    xrc.XRCCTRL(dialog, 'text').Wrap(300)
    dialog.Layout()
    dialog.Fit()
    username_ctrl = xrc.XRCCTRL(dialog, 'username')
    password_ctrl = xrc.XRCCTRL(dialog, 'password')
    username_ctrl.SetFocus()
    
    # These values don't come through in the xrc file
    username_ctrl.SetMinSize(wx.Size(200, -1))
    password_ctrl.SetMinSize(wx.Size(200, -1))
    if (dialog.ShowModal() != wx.ID_OK):
      return False

    username = username_ctrl.GetValue()
    password = password_ctrl.GetValue()
    
    invent = Invent(username, password)

    if (invent.login() == False):
      return self.login_to_invent()
    
    rospy.set_param('/invent/username', username)
    rospy.set_param('/invent/password', password)
    
    self._invent_client = invent
    return True
    

  ##\brief Loads inventory object, prompts for username/password if needed
  def get_inventory_object(self):
    if self._invent_client and self._invent_client.login():
      return self._invent_client

    username = rospy.get_param('/invent/username', None)
    password = rospy.get_param('/invent/password', None)

    if (username and password):
      invent = Invent(username, password)
      if (invent.login() == True):
        self._invent_client = invent
        return self._invent_client

      rospy.set_param('/invent/username', '')
      rospy.set_param('/invent/password', '')
    
    if not self.login_to_invent() or not self._invent_client or not self._invent_client.login():
      return None
    else:
      # self._invent_client is set by login_to_invent()
      return self._invent_client

  ##\brief Prompts user if they are sure they want to submit results
  def verify_submit(self):
    submit_check = 'Are you sure you want to submit?\n\n'
    submit_check += 'Press OK to submit or Cancel to recheck results.\n'
    submit_check += 'Be sure to select the correct directory to record your results.\n'

    are_you_sure = wx.MessageDialog(self, submit_check, 'Verify Results Submission',
                                    wx.OK|wx.CANCEL)
    return are_you_sure.ShowModal() == wx.ID_OK
    
  ##\brief Submits qualifications results to inventory, emails teams
  ##
  ## Uses result.py functions
  def submit_results(self, notes, directory):
    if not self.verify_submit():
      return

    self._waiting_for_submit = False

    if self._current_item is None:
      self.log('Can\'t submit, no item')
      self.reset()
      return

    self._results.set_results_dir(directory)

    invent = self.get_inventory_object()
    self._results.set_notes(notes)
    self._results.set_operator(rospy.get_param('invent/username', ''))

    self.log('Results logged to %s' % self._results.results_dir)
    res, log_str = self._results.log_results(invent)
    self.log(log_str)

    #if not res:
    #  wx.MessageBox('Unable to submit qualification results to invent. This may be a problem in the inventory system.\nMessage:%s' % log_str, 'Invent Submit Failed', wx.OK|wx.ICON_ERROR, self)
    
    if not self._results.email_qual_team():
      wx.MessageBox('Unable to email qualification results. Do you have \'sendmail\' installed?', 'Unable to email results', wx.OK|wx.ICON_ERROR, self)
      self.log('Unable to email summary.')
    else:
      self.log('Emailed summary to %s' % self._results.get_qual_team())
    
    self.reset()

  ##\brief Records test cancel, shuts down test
  def cancel(self, error = False):
    if self._results is not None:
      if error:
        self._results.error()
      else:
        self._results.cancel()
    
    self.test_finished()
  
  ##\brief Stops launches once we close window
  def on_close(self, event):
    if event.CanVeto() and (self.launches_running() or self._waiting_for_submit):
      dialog = wx.MessageDialog(self, 'Are you sure you want to close this window? This will shut down the current test and discard results. Press "Cancel" to resume.', 'Confirm Close', wx.OK|wx.CANCEL)
      if dialog.ShowModal() != wx.ID_OK:
        event.Veto()
        return

    event.Skip()
    
    self.stop_launches()
    self.Destroy()

