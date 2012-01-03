#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\author Josh Faust, Dave Hershberger

import os
import sys

from PySide.QtGui import *
from PySide.QtCore import *

PKG = 'qualification'
import roslib
roslib.load_manifest(PKG)

from optparse import OptionParser
#import shutil
#import glob
import traceback

import rospy

import rviz

from pr2_self_test_msgs.srv import ScriptDone, ScriptDoneRequest

from time import sleep

import threading

SRV_NAME = 'visual_check'
def call_done_service(result, msg):
  result_service = rospy.ServiceProxy(SRV_NAME, ScriptDone)
  r = ScriptDoneRequest()
  r.result = result
  r.failure_msg = msg
  try:
    rospy.wait_for_service(SRV_NAME, 5)
    result_service.call(r)
  except Exception, e:
    print >> sys.stderr, "Unable to call %s service. Make sure the service is up" % SRV_NAME
    

# # Calls the "OK" service after a timeout
# class VisualRunner(threading.Thread):
#   def __init__(self, app, timeout = None):
#     threading.Thread.__init__(self)
#     self.app = app
#     
#   def run(self):
#     start = rospy.get_time()
#     if timeout is None or timeout < 0:
#       return
#     while not rospy.is_shutdown():
#       if rospy.get_time() - start > timeout:
#         wx.CallAfter(self.app.frame.Close)
#         call_done_service(ScriptDoneRequest.RESULT_OK, 'Visual check passed by automatic runner.')
#         break
#       sleep(1.0)


class VisualizerFrame( QWidget ):
  def __init__( self, parent = None ):
    super(VisualizerFrame, self).__init__( parent )
    # title='Qualification Visualizer', size=(800, 600)
    
    self._visualizer_panel = rviz.VisualizationPanel()

    self._instructions = QTextBrowser()
    self._pass_button = QPushButton( "Pass" )
    self._fail_button = QPushButton( "Fail" )

    bottom_layout = QHBoxLayout()
    bottom_layout.addWidget( self._instructions )
    bottom_layout.addWidget( self._pass_button )
    bottom_layout.addWidget( self._fail_button )

    main_layout = QVBoxLayout()
    main_layout.addWidget( self._visualizer_panel )
    main_layout.addLayout( bottom_layout )

    self.setLayout( main_layout )
    
    self._pass_button.clicked.connect( self.on_pass )
    self._fail_button.clicked.connect( self.on_fail )

    self._shutdown_timer = QTimer( self )
    self._shutdown_timer.timeout.connect( self._on_shutdown_timer )
    self._shutdown_timer.start( 100 )
    
  def _on_shutdown_timer(self):
    if (rospy.is_shutdown()):
      self.close()
      
  def on_pass(self):
    call_done_service(ScriptDoneRequest.RESULT_OK, 'Visual check passed by operator.')
    self.close()

  def on_fail(self):
    call_done_service(ScriptDoneRequest.RESULT_FAIL, 'Visual check failed by operator.')
    self.close()
      
  def load_config_from_path(self, path):
    self._visualizer_panel.loadGeneralConfig(path)
    self._visualizer_panel.loadDisplayConfig(path)
    
  def set_instructions(self, instructions):
    self._instructions.setPlainText( instructions )
      
if __name__ == "__main__":
  if (len(sys.argv) < 2):
    call_done_service(ScriptDoneRequest.RESULT_ERROR, "Visual verifier not given path to config file.\nusage: visual_verifier.py 'path to config file'")
    print >> sys.stderr, 'Usage: visual_verifier.py \'path to config file\''
    sys.exit(1)

  filepath = sys.argv[1]

  if not os.path.exists( filepath ):
    call_done_service(ScriptDoneRequest.RESULT_ERROR, "Visual verifier not given path to actual config file.\nusage: visual_verifier.py 'path to config file'\nFile %s does not exist" % filepath )
    print >> sys.stderr, 'Usage: visual_verifier.py \'path to config file\'.\nGiven file does not exist'
    sys.exit(1)

  try:
    app = QApplication( sys.argv )
  
    frame = VisualizerFrame()
    # "Visual Verifier", wx.Size( 800, 600 ) )

    frame.load_config_from_path( filepath )
    frame.set_instructions( 'Move joints and verify robot is OK.' )
    frame.show()

    # VisualizationPanel's constructor does its own ros::init() in
    # C++, which sets the signal handler.  Here it is important to
    # call rospy.init_node() after that so the rospy signal handler
    # remains in effect instead.
    rospy.init_node('visual_verifier')

    # Uses this timeout to automatically pass visual verifier
    # Used in automated testing.
    timeout = rospy.get_param('visual_runner_timeout', -1)
    # if timeout > 0:
    #   runner = VisualRunner(app, timeout)
    #   runner.start()

    app.exec_()

    rospy.spin()

  except Exception, e:
    call_done_service(ScriptDoneRequest.RESULT_ERROR, 'Visual check recorded error: %s' % traceback.format_exc())
    rospy.logerr(traceback.format_exc())
