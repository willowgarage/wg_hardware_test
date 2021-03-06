#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

PKG = 'life_test'
import roslib
roslib.load_manifest(PKG)

import wx

import rospy

import sys, os

from roslaunch_caller import roslaunch_caller

import traceback

class TestManagerApp(wx.App):
    def OnInit(self, debug = False):

        args = rospy.myargv()
        debug = len(args) > 1 and args[1] == '--debug'

        try:
          self._core_launcher = roslaunch_caller.launch_core()
        except Exception, e:
            print >> sys.stderr, 'Failed to launch core. Another core may already be running.\n\n'
            wx.MessageBox('A ROS core is still running and preventing the Test Manager system from starting. Shut down ROS processes by using the "Kill ROS" icon.','ROS Already Running', wx.OK|wx.ICON_ERROR, None)
            traceback.print_exc()
            sys.exit(1)
            
        import life_test.result_dir 
        if not life_test.result_dir.check_results_dir():
            print >> sys.stderr, "Unable to write to results directory. Permissions invalid"
            wx.MessageBox("Unable to write to the \"~/wg_hardware_test/test_manager\" directory. Open a terminal and type, \"sudo chmod +rwx -R ~/wg_hardware_test/test_manager\" to fix the offending directory. You will have to restart Test Manager",
                          "Unable to Write Results", wx.OK|wx.ICON_ERROR, None)
            sys.exit(1)

        img_path = os.path.join(roslib.packages.get_pkg_dir(PKG), 'xrc', 'splash.jpg')
        
        bitmap = wx.Bitmap(img_path, type=wx.BITMAP_TYPE_JPEG)
        self._splash = wx.SplashScreen(bitmap, wx.SPLASH_CENTRE_ON_SCREEN, 30000, None, -1)

        rospy.init_node("Test_Manager")

        import life_test.manager

        self._frame = life_test.manager.TestManagerFrame(None, debug)
        self._frame.SetSize(wx.Size(1600, 1100))
        self._frame.Layout()
        self._frame.Centre()

        self._splash.Destroy()
        self._splash = None

        self._frame.Show(True)

        return True

    def OnExit(self):
        self._core_launcher.stop()

        if self._splash:
          self._splash.Destroy()

if __name__ == '__main__':
  try:
    app = TestManagerApp(0)
    app.MainLoop()
  except Exception, e:
    print >> sys.stderr, "Caught exception in TestManagerApp Main Loop"
    import traceback
    traceback.print_exc()
