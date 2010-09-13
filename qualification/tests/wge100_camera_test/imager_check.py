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

##\author Blaise Gassend
##\brief Displays prosilica focus on gnuplot window

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

class BayerDetector:
  def __init__(self, done_cb):
    self.done_cb = done_cb
    self.bridge = CvBridge()
    self.maxdim = -1
    self.bw_scaling = cv.CreateMat(2, 2, cv.CV_32FC1)
    self.bw_scaling[0,0] = 1
    self.bw_scaling[0,1] = 1
    self.bw_scaling[1,0] = 1
    self.bw_scaling[1,1] = 1
    self.col_scaling = cv.CreateMat(2, 2, cv.CV_32FC1)
    self.col_scaling[0,0] = .82
    self.col_scaling[0,1] = .91
    self.col_scaling[1,0] = .91
    self.col_scaling[1,1] = 1.36
    self.detected = False
    self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.process_image)

  def process_image(self, image):
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
            print x
            self.checker[0,x] = 1 if x % 2 else 0
            self.checker[1,x] = 1 - self.checker[0, x]
        self.maxdim = maxdim

    print "Maxdim", maxdim
    #print
    #for i in range(2):
    #    for j in range(maxdim):
    #        print self.checker[i, j],
    #    print
    
    rowmult = cv.GetSubRect(self.checker, (0, 0, height, 2))
    colmult = cv.GetSubRect(self.checker, (0, 0, width, 2))

    tmp = cv.CreateMat(height, 2, cv.CV_32FC1)
    cv.GEMM(origimg, colmult, 1, None, 0, tmp, cv.CV_GEMM_B_T)

    out = cv.CreateMat(2, 2, cv.CV_32FC1)
    cv.GEMM(rowmult, tmp, 1, None, 0, out, 0)

    if self.try_match(out, self.bw_scaling):
        self.is_color = False
    print "BW: ", self.status, 
    if self.try_match(out, self.col_scaling):
        self.is_color = True
    print "Col: ", self.status

    if self.detected:
        self.done_cb()

    # FIXME Update status.

  def try_match(self, out, scaling):
      tmp = cv.CreateMat(2, 2, cv.CV_32FC1)
      cv.Mul(out, scaling, tmp)
      avg, dev = cv.AvgSdv(tmp)
      avg = avg[0]
      dev = dev[0]
      print
      for i in range(2):
          for j in range(2):
              print out[i, j],
          print
      print
      for i in range(2):
          for j in range(2):
              print tmp[i, j],
          print
      print
      print avg, dev, dev/avg
      if avg < 16:
          self.status = "Too dark.",avg
      elif avg > 128:
          self.status = "Too bright.", avg
      elif dev / avg < 0.05:
          self.status = "Match!"
          self.detected = True
          return True
      else:
          self.status = "Not recognized."
      return False

def main(args):
  rospy.init_node('imager_checker')
  bd = None
  md = None
  def done_cb():
      if not bd:
          return
      if not bd.detected:
          return
      if not md:
          return
      if not md.detected:
          return
      print "Ready!!"
  bd = BayerDetector(done_cb)
  #md = ModelDetector(done_cb)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  except:
    print "Shut down"

if __name__ == '__main__':
    main(sys.argv)
