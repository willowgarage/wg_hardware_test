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

##\author Kevin Watts
##\brief Loads qualification GUI for component qualification

PKG = 'qualification'
import roslib
roslib.load_manifest(PKG)

from roslaunch_caller import roslaunch_caller 

import wx
import sys, os
import rospy
import traceback

## Starts roscore, qualification app for components
class QualificationApp(wx.App):
    def OnInit(self):
        try:
            self._core_launcher = roslaunch_caller.launch_core()
        except Exception, e:
            print >> sys.stderr, 'Failed to launch core. Another core may already be running.\n\n'
            wx.MessageBox('A ROS core is still running and preventing the qualification system from starting. Shut down ROS processes by using the "Kill ROS" icon.','ROS Already Running', wx.OK|wx.ICON_ERROR, None)
            traceback.print_exc()
            sys.exit(1)

        # Check that we can write our results into temporary, permenant files
        from qualification.result_dir import check_qual_result_dir, check_qual_temp_dir
        if not check_qual_temp_dir():
            wx.MessageBox("Unable to write to the temporary results directory. This will cause weird problems. Open a terminal and type, \"sudo rm /tmp/* -rf\" to remove the offending directory. You will have to restart the qualification system.", 
                          "Unable to Write Results", wx.OK|wx.ICON_ERROR, None)
            sys.exit(1)

        if not check_qual_result_dir():
            wx.MessageBox("Unable to write to the \"~/wg_hardware_test/qualification\" directory. Open a terminal and type, \"sudo chmod +rwx -R ~/wg_hardware_test/qualification\" to fix the offending directory. You will have to restart the qualification system.", 
                          "Unable to Write Results", wx.OK|wx.ICON_ERROR, None)
            sys.exit(1)

        
        rospy.init_node("qualification")
        
        img_path = os.path.join(roslib.packages.get_pkg_dir(PKG), 'xrc', 'splash.jpg')
        
        bitmap = wx.Bitmap(img_path, type=wx.BITMAP_TYPE_JPEG)
        self._splash = wx.SplashScreen(bitmap, wx.SPLASH_CENTRE_ON_SCREEN, 30000, None, -1)
        
        import qualification.component_qual

        qual_options = qualification.component_qual.ComponentQualOptions()
        if len(sys.argv) > 1 and sys.argv[1] == '--debug':
            qual_options.debug = True

        self._frame = qualification.component_qual.ComponentQualFrame(None, qual_options)
        self._frame.SetSize(wx.Size(700,1000))
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
    os.environ['ROSLAUNCH_SSH_UNKNOWN'] = '1'
    try:
        app = QualificationApp()
        app.MainLoop()
    except Exception, e:
        print >> sys.stderr, "Caught exception in Qualification App Main Loop"
        import traceback
        traceback.print_exc()
