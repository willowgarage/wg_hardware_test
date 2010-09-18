#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008-2010, Willow Garage, Inc.
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

##\author Blaise Gassend
##\brief Checks that the imager model and color filter on a wge100 camera
##match its serial number.

from __future__ import with_statement

PKG = 'qualification'
import roslib
roslib.load_manifest(PKG)
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import subprocess
from time import sleep
from pr2_self_test_msgs.srv import *
from diagnostic_msgs.msg import *
import wx
import time

class BayerDetector:
  def __init__(self, done_cb, red_source):
    self.status = "Waiting for first image."
    self.done_cb = done_cb
    self.red_source = red_source
    self.bridge = CvBridge()
    self.maxdim = -1
    self.bw_scaling = cv.CreateMat(2, 2, cv.CV_32FC1)
    self.bw_scaling[0,0] = 1
    self.bw_scaling[0,1] = 1
    self.bw_scaling[1,0] = 1
    self.bw_scaling[1,1] = 1
    self.col_scaling = cv.CreateMat(2, 2, cv.CV_32FC1)
    if red_source:
        self.col_scaling[0,0] = .50
        self.col_scaling[0,1] = 1.04
        self.col_scaling[1,0] = 1.04
        self.col_scaling[1,1] = 1.40
    else:
        self.col_scaling[0,0] = .82
        self.col_scaling[0,1] = .91
        self.col_scaling[1,0] = .91
        self.col_scaling[1,1] = 1.36
    self.detected = False
    self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.process_image)

  def process_image(self, image):
        wx.CallAfter(self.process_image_main_thread, image)

  def process_image_main_thread(self, image):
    try:
      image.encoding = "mono8" # Don't trust what the driver is telling us.
      cv_image = self.bridge.imgmsg_to_cv(image, "mono8")
    except CvBridgeError, e:
      import traceback
      traceback.print_exc()

    (width, height) = cv.GetSize(cv_image)
    
    origimg = cv.CreateMat(height, width, cv.CV_32FC1)
    cv.Scale(cv_image, origimg, 4.0 / (width * height), 0.0)
    
    maxdim = max(width, height)
    if maxdim > self.maxdim:
        # Prepare the checkerboard matrix that will extract even/odd
        # rows/columns.
        self.checker = cv.CreateMat(2, maxdim, cv.CV_32FC1)
        for x in range(maxdim):
            self.checker[0,x] = 1 if x % 2 else 0
            self.checker[1,x] = 1 - self.checker[0, x]
        self.maxdim = maxdim

    rowmult = cv.GetSubRect(self.checker, (0, 0, height, 2))
    colmult = cv.GetSubRect(self.checker, (0, 0, width, 2))

    tmp = cv.CreateMat(height, 2, cv.CV_32FC1)
    cv.GEMM(origimg, colmult, 1, None, 0, tmp, cv.CV_GEMM_B_T)

    out = cv.CreateMat(2, 2, cv.CV_32FC1)
    cv.GEMM(rowmult, tmp, 1, None, 0, out, 0)

    if self.try_match(out, self.bw_scaling):
        self.detected_type = 'mono'
        self.status = "mono"
        self.detected = True
    print "BW: ", self.status, 
    if self.try_match(out, self.col_scaling):
        self.detected_type = 'color'
        self.status = "color"
        self.detected = True
    print "Col: ", self.status

    if self.detected:
        self.done_cb()
        self.image_sub.unregister()

    # FIXME Update status.

  def try_match(self, out, scaling):
      tmp = cv.CreateMat(2, 2, cv.CV_32FC1)
      cv.Mul(out, scaling, tmp)
      avg, _ = cv.AvgSdv(tmp)
      avg = avg[0]
      dev = 0
      for x in range(2):
          for y in range(2):
              dev = max(dev, abs(tmp[x,y] - avg))
      print
      for i in range(2):
          for j in range(2):
              print tmp[i, j],
          print
      print
      print avg, dev, dev/avg
      if avg < 16:
          self.status = "Too dark. (%i)"%int(avg)
      elif avg > 192:
          self.status = "Too bright. (%i)"%int(avg)
      elif dev / avg < 0.1:
          self.status = "Match!"
          return True
      else:
          if self.red_source:
              self.status = "Camera should be pointing at the red LED."
          else:
              self.status = "Point camera at white target."
      return False

class ModelDetector:
    def __init__(self, done_cb):
        self.status = "Waiting for model number."
        self.model = None
        self.done_cb = done_cb
        rospy.Subscriber('/diagnostics', DiagnosticArray, self.msg_cb)

    def msg_cb(self, msg):
        wx.CallAfter(self.msg_cb_main_thread, msg)

    def msg_cb_main_thread(self, msg):
        for s in msg.status:
            if s.name == 'wge100_camera: Driver Status':
                for kv in s.values:
                    if kv.key == 'Imager model':
                        self.model = kv.value
                        self.status = "Found model: %s"%self.model
                        self.done_cb()

acceptable_combinations = [
        ('6805018', 'color', 'MT9V032'),
        ('6805027', 'mono', 'MT9V034'),
        ('6805030', 'color', 'MT9V032'),
        ]

class MainWindow(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, wx.ID_ANY, "WGE100 imager check", 
                pos=(200, 200), size=(400, 200))
        sizer = wx.FlexGridSizer(0, 2, vgap = 1, hgap = 4)
        sizer.SetFlexibleDirection(wx.BOTH)
        sizer.AddGrowableCol(1,1)
        
        sizer.Add(wx.StaticText(self, wx.ID_ANY, label="Imager model:"))
        self.imager_text = wx.TextCtrl(self, wx.ID_ANY, value="Unknown.")
        self.imager_text.SetEditable(False)
        sizer.Add(self.imager_text, 1, wx.ALL | wx.EXPAND)
        
        sizer.Add(wx.StaticText(self, wx.ID_ANY, label="Detected color filter:"))
        self.color_text = wx.TextCtrl(self, wx.ID_ANY, value="Unknown.")
        self.color_text.SetEditable(False)
        sizer.Add(self.color_text, 1, wx.ALL | wx.EXPAND)
        
        sizer.Add(wx.StaticText(self, wx.ID_ANY, label="Serial number prefix:"))
        self.barcode_text = wx.TextCtrl(self, wx.ID_ANY, value="Unknown.")
        self.barcode_text.SetEditable(False)
        self.barcode_text.SetBackgroundColour((0, 255, 0))
        sizer.Add(self.barcode_text, 1, wx.ALL | wx.EXPAND)
        
        self.SetSizerAndFit(sizer)
        self.SetMinSize((self.GetSize()[0] + 200, 10))
        sizer.Layout()
        self.Show()

def main(args):
  bd = None
  md = None
  result_sent = []
  
  def update_ui():
      frame.color_text.SetValue(bd.status)
      if not bd.detected:
          frame.color_text.SetBackgroundColour((255,0,0))
      else:
          frame.color_text.SetBackgroundColour((0,255,0))
      frame.imager_text.SetValue(md.status)
      if not md.model:
          frame.imager_text.SetBackgroundColour((255,0,0))
      else:
          frame.imager_text.SetBackgroundColour((0,255,0))
      frame.barcode_text.SetValue(barcode_prefix)
  
  def done_cb():
      global app
      if result_sent:
          return
 
      update_ui()
      
      if not bd:
          return
      if not bd.detected:
          return
      if not md.model:
          return
 
      r = TestResultRequest()
      if (barcode_prefix, bd.detected_type, md.model) in acceptable_combinations:
          r.text_summary = "Found acceptable imager."
          r.html_result = "<p>Test passed.</p>"
          r.result = TestResultRequest.RESULT_PASS
      else:
          r.text_summary = "Unacceptable imager."
          r.html_result = "<p>Test failed.</p>"
          r.result = TestResultRequest.RESULT_FAIL
      r.html_result += "<p>Imager model: %s</p>"%md.model 
      r.html_result += "<p>Detected color filter: %s</p>"%bd.detected_type 
      r.html_result += "<p>Serial number prefix: %s</p>"%barcode_prefix
      r.html_result += "<p></p>"
      r.html_result += "<p>Acceptable combinations:</p>"
      for barcode, filter, imager in acceptable_combinations:
          r.html_result += "<p>%s, %s, %s</p>"%(barcode, filter, imager)
      
      rospy.loginfo("Test completed, waiting for result server.")
      result_service = rospy.ServiceProxy('test_result', TestResult)
      rospy.wait_for_service('test_result')
      result_service.call(r)
      result_sent.append(None)
      time.sleep(1) # Just so the user has time to see the UI.
      rospy.on_shutdown(app.AddPendingEvent(wx.ID_EXIT))
      app.Exit()
      print "Exit requested"
  
  rospy.init_node('imager_checker')
  app = wx.PySimpleApp()
  rospy.on_shutdown(app.Exit)
  barcode_prefix = str(rospy.get_param('qual_item/serial'))[0:7]
  frame = MainWindow()
  bd = BayerDetector(done_cb, rospy.get_param('~red_source'))
  md = ModelDetector(done_cb)
  update_ui()
  try:
      app.MainLoop()
  except KeyboardInterrupt:
    print "Shutting down"
  except:
    rospy.signal_shutdown("Shutting down.")
    print "Shut down"

if __name__ == '__main__':
    main(sys.argv)
