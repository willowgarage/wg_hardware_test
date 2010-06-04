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
##\brief Kills ROS processes using kkill

PKG = 'pr2_hw_test_utilities'

import wx
import sys, subprocess, os


from time import sleep

my_pid = str(os.getppid())
print my_pid

app = wx.PySimpleApp()

dialog = wx.MessageDialog(None, "Are you are you want to kill ROS? This will kill the Qualification system and Test Manager. Press \"Cancel\" to abort.", "Confirm Kill ROS", wx.OK|wx.CANCEL)
if dialog.ShowModal() != wx.ID_OK:
    sys.exit(1)

#cmd = [ 'kkill' ]

p = subprocess.Popen('sudo kkill %s' % my_pid, stdout = subprocess.PIPE, 
                     stderr = subprocess.PIPE, shell=True)
retcode = p.returncode

if retcode != 0:
    wx.MessageBox("Unable to kill ROS. Retry by pressing \"Kill ROS\".", "Unable to kill ROS", wx.OK)
else:
    wx.MessageBox("ROS processes killed. Close any remaining windows", "ROS Killed", wx.OK)

