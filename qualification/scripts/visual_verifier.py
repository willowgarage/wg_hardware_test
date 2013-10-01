#!/usr/bin/env python
#
# Copyright (c) 2008, Willow Garage, Inc.
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

##\author Josh Faust

import os
import sys

PKG = 'qualification'
import roslib
roslib.load_manifest(PKG)

from optparse import OptionParser
import traceback

import rospy

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import Qt,QTimer
import rviz

from pr2_self_test_msgs.srv import ScriptDone, ScriptDoneRequest

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
    

# Calls the "OK" service after a timeout
class VisualRunner(threading.Thread):
  def __init__(self, app, timeout = None):
    threading.Thread.__init__(self)
    self.app = app
    self.timeout = timeout
    
  def run(self):
    start = rospy.get_time()
    if self.timeout is None or self.timeout < 0:
      return
    while not rospy.is_shutdown():
      if rospy.get_time() - start > self.timeout:
        call_done_service(ScriptDoneRequest.RESULT_OK, 'Visual check passed by automatic runner.')
        self.app.closeAllWindows()
        break
      rospy.sleep(1.0)


class VisualizerFrame:
  def __init__(self, title='Qualification Visualizer [*]', size=(800, 600)):
    visualizer_frame = rviz.VisualizationFrame()
    visualizer_frame.initialize()
    visualizer_frame.setWindowTitle(title)
    visualizer_frame.resize(*size)
    
    self._visualizer_frame = visualizer_frame
    
    myWidget = QWidget(visualizer_frame)
    layout = QGridLayout(myWidget)
    self._instructions_ctrl = QTextEdit(myWidget)
    self._instructions_ctrl.setReadOnly(True)
    self._pass_button = QPushButton("&Pass", myWidget)
    self._fail_button = QPushButton("&Fail", myWidget)
    layout.addWidget(self._instructions_ctrl, 0, 0, 1, 1)
    layout.addWidget(self._pass_button, 0, 1, 1, 1)
    layout.addWidget(self._fail_button, 0, 2, 1, 1)
    
    visualizer_frame.addPane(title, myWidget, Qt.BottomDockWidgetArea)
    
    self._pass_button.clicked.connect(self.on_pass)
    self._fail_button.clicked.connect(self.on_fail)
    
    #  timer fires every 100 ms
    self._shutdown_timer = QTimer()
    self._shutdown_timer.timeout.connect(self._on_shutdown_timer)
    self._shutdown_timer.start(100)
    
  def _on_shutdown_timer(self, event):
    if (rospy.is_shutdown()):
      self._visualizer_frame.close()
      
  def on_pass(self, event):
    call_done_service(ScriptDoneRequest.RESULT_OK, 'Visual check passed by operator.')
    self._visualizer_frame.close()

  def on_fail(self, event):
    call_done_service(ScriptDoneRequest.RESULT_FAIL, 'Visual check failed by operator.')
    self._visualizer_frame.close()
      
  def load_config_from_path(self, path):
    manager = self._visualizer_frame.getManager()
    manager.removeAllDisplays()
    self._visualizer_frame.loadDisplayConfig(path)
    
  def set_instructions(self, instructions):
    self._instructions_ctrl.setText(instructions)

  def show(self):
     self._visualizer_frame.show()
      
class VisualizerApp():
  def __init__(self, file):
    self._filepath = file
    self._instructions = 'Move joints and verify robot is OK.'
    
    try:
      self.frame = VisualizerFrame("Visual Verifier [*]", ( 800, 600 ) )
    
      if (not os.path.exists(self._filepath)):
        call_done_service(ScriptDoneRequest.RESULT_ERROR, 'Visual check recorded error, file does not exist!')
        rospy.logerr('Visual check recorded error, file does not exist!')
        return False
    
      self.frame.load_config_from_path(self._filepath)
      self.frame.set_instructions(self._instructions)
      self.frame.show()
      return True
    except:
      traceback.print_exc()
      rospy.logerr('Error initializing rviz: %s' % traceback.format_exc())
      call_done_service(ScriptDoneRequest.RESULT_ERROR, 'Visual check recorded error on initialization: %s' % traceback.format_exc())
      return False

if __name__ == "__main__":
  if (len(sys.argv) < 2):
    call_done_service(ScriptDoneRequest.RESULT_ERROR, "Visual verifier not given path to config file.\nusage: visual_verifier.py 'path to config file'")
    print >> sys.stderr, 'Usage: visual_verifier.py \'path to config file\''
    sys.exit(1)

  if not os.path.exists(sys.argv[1]):
    call_done_service(ScriptDoneRequest.RESULT_ERROR, "Visual verifier not given path to actual config file.\nusage: visual_verifier.py 'path to config file'\nFile %s does not exist" % sys.argv[1])
    print >> sys.stderr, 'Usage: visual_verifier.py \'path to config file\'.\nGiven file does not exist'
    sys.exit(1)

  try:
    app = QApplication(sys.argv)
    v_app = VisualizerApp(sys.argv[1])
    rospy.init_node('visual_verifier')

    # Uses this timeout to automatically pass visual verifier
    # Used in automated testing.
    #  TODO: fix for QT and restore
    #timeout = rospy.get_param('visual_runner_timeout', -1)
    #if timeout > 0:
    #  runner = VisualRunner(app, timeout)
    #  runner.start()

    exit_code = app.exec_()

    # Give ROS a chance to clean up before we exit
    rospy.spin()
    sys.exit(exit_code)
  except KeyboardInterrupt:
    pass
  except Exception, e:
    call_done_service(ScriptDoneRequest.RESULT_ERROR, 'Visual check recorded error: %s' % traceback.format_exc())
    rospy.logerr(traceback.format_exc())
