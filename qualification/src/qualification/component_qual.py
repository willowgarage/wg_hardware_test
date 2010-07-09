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

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

import rospy

import os
import sys
import socket
import wx
import time
from wx import xrc
from wx import html

from xml.dom import minidom

from qualification.test import *
from qualification.qual_frame import *

from qualification.test_loader import load_configs_from_map, load_tests_from_map, load_wg_station_map

TESTS_DIR = os.path.join(roslib.packages.get_pkg_dir(PKG), 'tests')
CONFIG_DIR = os.path.join(roslib.packages.get_pkg_dir(PKG), 'config')

class ConfigObject(QualTestObject):
  def __init__(self, name, serial):
    QualTestObject.__init__(self, name, serial)
    self._config = True

class ComponentQualOptions(QualOptions):
  def __init__(self):
    QualOptions.__init__(self)

## Allows user to choose which component to test and which test to run
class SerialPanel(wx.Panel):
  ##@param parent wxPanel: Parent frame
  ##@param resource: XRC resource to load panel
  ##@param qualification_frame: Qualfication frame to pass test, serial to
  def __init__(self, parent, resource, qualification_frame):
    wx.Panel.__init__(self, parent)
    
    self._manager = qualification_frame
    
    self._res = resource

    self._tests = {}
    self._debugs = [] # Hold any SN that can run in debug mode
    self._configs = {}
    loaded_ok = self.load_test_config_files()
    
    self._panel = resource.LoadPanel(self, 'serial_panel')
    self._sizer = wx.BoxSizer(wx.HORIZONTAL)
    
    self._sizer.Add(self._panel, 1, wx.EXPAND|wx.ALIGN_CENTER_VERTICAL)
    self.SetSizer(self._sizer)
    self.Layout()
    self.Fit()

    # Component qualification tab
    self._test_button = xrc.XRCCTRL(self._panel, 'test_button')
    self._serial_text = xrc.XRCCTRL(self._panel, 'serial_text')
    
    self._test_button.Bind(wx.EVT_BUTTON, self.on_test)
        
    # Config tab
    self._serial_text_conf = xrc.XRCCTRL(self._panel, 'serial_text_conf')

    self._config_button = xrc.XRCCTRL(self._panel, 'config_button')
    self._config_button.Bind(wx.EVT_BUTTON, self.on_config)

    # Starts on test cart panel
    self._notebook = xrc.XRCCTRL(self._panel, 'test_tab_control')
    self._notebook.ChangeSelection(0)

    self._panel.Bind(wx.EVT_CHAR, self.on_char)
    self._serial_text.SetFocus()

    if not loaded_ok:
      wx.MessageBox("Warning: Unable to load tests configuration files. May be corrupt. Revert any changes you have made to these files and retry", "Invalid Configuration Files", wx.OK|wx.ICON_ERROR, self)

  
  def load_test_config_files(self):
    """
    Called at startup or by menu option. Loads configuration files that define tests.
    """
    tests = {}
    debugs = [] # Hold any SN that can run in debug mode
    try:
      tests_ok = load_tests_from_map(tests, debugs) 
      if tests_ok:
        self._debugs = debugs
        self._tests = tests
    except Exception, e:
      tests_ok = False

    configs = {}
    try:
      configs_ok = load_configs_from_map(self._configs)
      if configs_ok:
        self._configs = configs
    except Exception, e:
      configs_ok = False

    return tests_ok and configs_ok

  def _is_debug_test(self, serial):
    return serial[0:7] in self._debugs

  ##\brief Checks that serial number is valid using invent_client
  def _check_serial_input(self, serial):
    if self._manager.options.debug:
      return True

    if self._is_debug_test(serial):
      return True

    iv = self._manager.get_inventory_object()
    if not iv or not iv.login():
      wx.MessageBox("Unable to check serial number. Unable to login in Inventory system", "Error - Unable to check serial number", wx.OK|wx.ICON_ERROR, self)
      return False

    return iv.check_serial_valid(serial)

  def on_config(self, event):
    # Get selected launch file
    serial = str(self._serial_text_conf.GetValue())

    if not self._check_serial_input(serial):
      wx.MessageBox('Invalid serial number, unable to configure. Check serial number and retry.','Error - Invalid serial number', wx.OK|wx.ICON_ERROR, self)
      return

    if not self.has_conf_script(serial):
      wx.MessageBox('No configuration procedure defined for that serial number.','Error - Invalid serial number', wx.OK|wx.ICON_ERROR, self)
      return

    config_test = self.select_conf_to_load(serial)
    if config_test is None:
      return

    if not config_test.validate():
      wx.MessageBox('Unable to load configuration data and parameters. Check file and try again.','Failed to load test', wx.OK|wx.ICON_ERROR, self)
      return 
    
    item = ConfigObject(config_test.get_name(), serial)

    self._manager.begin_test(config_test, item)

  def has_conf_script(self, serial):
    return self._configs.has_key(serial[0:7])

  def has_test(self, serial):
    return self._tests.has_key(serial[0:7])

  def on_test(self, event):
    # Get the first 7 characters of the serial
    serial = self._serial_text.GetValue()
    
    if not self._check_serial_input(serial):
      wx.MessageBox('Invalid serial number, unable to test component. Check serial number and retry.','Error - Invalid serial number', wx.OK|wx.ICON_ERROR, self)
      return

    if not self.has_test(serial):
      wx.MessageBox('No test defined for that serial number.','Error - Invalid serial number', wx.OK|wx.ICON_ERROR, self)
      return

    short_serial = serial[0:7]

    my_test = self.select_test_to_load(short_serial)
    
    if my_test is None:
      return

    if not my_test.validate():
      wx.MessageBox('Unable to load test data and parameters. Check file %s and try again.' % test_file,'Failed to load test', wx.OK|wx.ICON_ERROR, self)
      return 

    # Item
    item = QualTestObject(my_test.get_name(), serial)
  
    self._manager.begin_test(my_test, item)
 
  # Select if we have multiple, etc
  def select_string_from_list(self, msg, lst):
    if len(lst) == 1:
      return lst[0]

    # Load robot selection dialog
    dialog = self._res.LoadDialog(self, 'select_test_dialog')
    select_text = xrc.XRCCTRL(dialog, 'select_text')
    select_text.SetLabel(msg)
    test_box = xrc.XRCCTRL(dialog, 'test_list_box')
    test_box.InsertItems(lst, 0)
    test_box.SetSelection(0)
    
    select_text.Wrap(270)

    dialog.Layout()

    # return string of test folder/file to run
    if (dialog.ShowModal() == wx.ID_OK):
      desc = test_box.GetStringSelection()
      dialog.Destroy()
      return desc
    else: 
      dialog.Destroy()
      return None

  def select_conf_to_load(self, serial):
    short_serial = serial[0:7]
    
    # Load select_test_dialog
    configs_by_descrip = {}
    for conf in self._configs[short_serial]:
      configs_by_descrip[conf.get_name()] = conf
      
    msg = 'Select configuration type'

    descrips = configs_by_descrip.keys()
    descrips.sort()
  
    choice = self.select_string_from_list(msg, descrips)
    if choice is None:
      return None
    return configs_by_descrip[choice]
    
  # If more than one test for that serial, calls up prompt to ask user to select
  def select_test_to_load(self, short_serial):
    # Load select_test_dialog
    tests_by_descrip = {}
    for test in self._tests[short_serial]:
      tests_by_descrip[test.get_name()] = test
    
    descrips = tests_by_descrip.keys()
    descrips.sort()
    
    msg = 'Select component or component type to qualify.'

    choice = self.select_string_from_list(msg, descrips)
    if choice is None:
      return None
    return tests_by_descrip[choice]

  def on_char(self, event):
    # 347 is the keycode sent at the beginning of a barcode
    if (event.GetKeyCode() == 347):
      # Clear the old contents and put focus in the serial box so the rest of input goes there
      self._serial_text.Clear()
      self._serial_text_conf.Clear()

class ComponentQualFrame(QualificationFrame):
  def __init__(self, parent, options):
    self._serial_panel = None
    QualificationFrame.__init__(self, parent, options)
    
    self.load_wg_test_map()
    self.create_menubar()

  def get_loader_panel(self):
    if not self._serial_panel:
      self._serial_panel = SerialPanel(self._top_panel, self._res, self)
    return self._serial_panel
  
  ##\todo Overloaded in subclasses
  def create_menubar(self):
    menubar = wx.MenuBar()
    self._file_menu = wx.Menu()
    self._file_menu.Append(1001, "Invent Login\tCTRL+l")
    self._file_menu.Append(1002, "R&eload Test/Config Lists")
    self._file_menu.Append(1003, "Reload W&G Station Map")
    self._file_menu.Append(wx.ID_EXIT, "E&xit")
    menubar.Append(self._file_menu, "&File")

    self._options_menu = wx.Menu()
    self._options_menu.AppendCheckItem(2001, "Always Show Results")
    menubar.Append(self._options_menu, "&Options")

    self._powerboard_menu = wx.Menu()
    self._powerboard_menu.Append(3101, "Power Board\tCTRL+b")
    self._powerboard_menu.AppendCheckItem(3150, "Breaker 0\tCTRL+0")
    self._powerboard_menu.AppendCheckItem(3151, "Breaker 1\tCTRL+1")
    self._powerboard_menu.AppendCheckItem(3152, "Breaker 2\tCTRL+2")
    self._powerboard_menu.Check(3150, rospy.get_param('/qualification/powerboard/0'))
    self._powerboard_menu.Check(3151, rospy.get_param('/qualification/powerboard/1'))
    self._powerboard_menu.Check(3152, rospy.get_param('/qualification/powerboard/2'))

    menubar.Append(self._powerboard_menu, "&Powerboard")

    self._host_menu = wx.Menu()
    self._host_menu.Append(4001, "Select Test Host\tCTRL+h")
    menubar.Append(self._host_menu, "Test &Host")

    self._debug_menu = None
    if self.options.debug:
      self._debug_menu = wx.Menu()
      self._debug_menu.Append(5005, "Abort Subtest")
      menubar.Append(self._debug_menu, "Debug Mode")

    self.SetMenuBar(menubar)
    self.Bind(wx.EVT_MENU, self.on_menu)

  def get_powerboard(self):
    while not rospy.is_shutdown():
      # Get powerboard from user
      serial = wx.GetTextFromUser('Enter last four digits of power board serial number', 'Select Power Board', rospy.get_param('/qualification/powerboard/serial', ''))
      if len(serial) == 0:
        return
      if len(serial) == 4 and unicode(serial).isnumeric():
        rospy.set_param('/qualification/powerboard/serial', serial)
        return
      are_you_sure = wx.MessageDialog(self, 'Invalid powerboard ID. Retry?', 'Invalid serial',
                                      wx.OK|wx.CANCEL)
      if are_you_sure.ShowModal() != wx.ID_OK:
        return

  def on_menu(self, event):
    if (event.GetEventObject() == self._file_menu):
      if (event.GetId() == wx.ID_EXIT):
        self.Close()
        return
      if (event.GetId() == 1001):
        self.login_to_invent()
        return
      if (event.GetId() == 1002):
        if not self._serial_panel.load_test_config_files():
          wx.MessageBox('Configuration files for loading tests are invalid. Check the files "tests/tests.xml" and "config/configs.xml" and retry.',
                        'Invalid Configuration Files', wx.OK|wx.ICON_ERROR, self)
        else:
          wx.MessageBox('Reloaded configuration files successfully.',
                        'Reloaded Configuration Files', wx.OK, self)
        return
      if (event.GetId() == 1003):
        if not self.load_wg_test_map():
          wx.MessageBox('Configuration file for loading WG station map is invalid. Check the file wg_map.xml" and retry.',
                        'Invalid Configuration File', wx.OK|wx.ICON_ERROR, self)
        else:
          wx.MessageBox('Reloaded WG station map configuration file successfully.',
                        'Reloaded Configuration File', wx.OK, self)
        return

    if (event.GetEventObject() == self._options_menu):
      if (event.GetId() == 2001):
        self.options.always_show_results = self._options_menu.IsChecked(2001)

    if (event.GetEventObject() == self._powerboard_menu):
      if (event.GetId() == 3101):
        self.get_powerboard()
      
      if (event.GetId() == 3150):
        rospy.set_param('/qualification/powerboard/0', self._powerboard_menu.IsChecked(3150))
      if (event.GetId() == 3151):
        rospy.set_param('/qualification/powerboard/1', self._powerboard_menu.IsChecked(3151))
      if (event.GetId() == 3152):
        rospy.set_param('/qualification/powerboard/2', self._powerboard_menu.IsChecked(3152))

    if (event.GetEventObject() == self._host_menu):
      if (event.GetId() == 4001):
        self.set_test_host()

    if (event.GetEventObject() == self._debug_menu):
      if (event.GetId() == 5005):
        self.abort_active_test(True)

  def abort_active_test(self, can_veto):
    if can_veto:
      are_you_sure = wx.MessageDialog(self, "Are you sure you want to abort the current operation?", 
                                      "Confirm Abort", wx.OK|wx.CANCEL)
      if are_you_sure.ShowModal() != wx.ID_OK:
        return

    if self._subtest_launch is not None:
      self.log('Aborting subtest')
      self.subtest_finished(subtest_timeout(0))
      return

    if self._prestartup_launch is not None:
      self.log('Aborting pretest')
      self.prestartup_finished(script_timeout(0))
      return

    if self._shutdown_launch is not None:
      self.log('Aborting shutdown script')
      self.shutdown_finished(script_timeout(0))

    wx.MessageBox('No subtests, shutdown scripts or pretests to abort. Press the "Cancel" button to terminate test', 'Unable to abort', wx.OK)
    

  def set_test_host(self):
    curr_host = os.environ['ROS_TEST_HOST']
    
    while not rospy.is_shutdown():
      host = wx.GetTextFromUser('Enter test host', 'Test Host', curr_host)
      
      if host == '':
        wx.MessageBox('Host name unchanged. Current hostname: "%s".' % curr_host, 'Test Host Unchanged', wx.OK)
        break

      try:
        machine_addr = socket.gethostbyname(host)
        os.environ['ROS_TEST_HOST'] = host
        break
      except socket.gaierror:
        wx.MessageBox('Hostname %s is invalid. Try again or click "Cancel".' % host, 'Test Host Invalid', wx.OK)
        
  def load_wg_test_map(self):
    wgstations = {}
    load_ok = load_wg_station_map(wgstations)

    gui_name = socket.gethostname()

    # Sets default host, powerboard to None
    rospy.set_param('/qualification/powerboard/serial', '0000')
    rospy.set_param('/qualification/powerboard/0', False)
    rospy.set_param('/qualification/powerboard/1', False)
    rospy.set_param('/qualification/powerboard/2', False)
    os.environ['ROS_TEST_HOST'] = gui_name
    
    if not load_ok:
      wx.MessageBox('Error: Unable to parse \'qualification/wg_map.xml\'. Please check the document and retry.','Unable to parse configuration', wx.OK|wx.ICON_ERROR, self)

      return False

    if not gui_name in wgstations:
      wx.MessageBox('Warning: Host %s not found in list of known hosts. Check file: \'qualification/wg_map.xml\'. You may be unable to run certain qualification tests' % gui_name,'Host not found', wx.OK|wx.ICON_ERROR, self)
      return False

    my_station = wgstations[gui_name]
    rospy.set_param('/qualification/powerboard/serial', my_station.powerboard)
    rospy.set_param('/qualification/powerboard/0', my_station.breaker0)
    rospy.set_param('/qualification/powerboard/1', my_station.breaker1)
    rospy.set_param('/qualification/powerboard/2', my_station.breaker2)

    try:
      machine_addr = socket.gethostbyname(my_station.test_host)
      os.environ['ROS_TEST_HOST'] = my_station.test_host
    except socket.gaierror:
      wx.MessageBox('Unable to resolve remote test host %s. The file: \'qualification/wg_map.xml\' may be invalid.' % my_station.test_host, 'Remote Host Not Found', wx.OK|wx.ICON_ERROR, self)
      return False

    return True
    



      
      



