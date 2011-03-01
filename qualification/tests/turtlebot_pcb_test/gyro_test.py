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

import roslib
roslib.load_manifest('qualification')

import sys
import rospy
from std_srvs.srv import Empty
from pr2_self_test_msgs.srv import TestResult, TestResultRequest
from pr2_self_test_msgs.msg import Plot, TestParam, TestValue
from turtlebot_node.srv import SetDigitalOutputsRequest, SetDigitalOutputs
import diagnostic_msgs.msg
import math

def test():
    rospy.init_node("gyro_test")

    agg_sub = rospy.Subscriber('diagnostics', diagnostic_msgs.msg.DiagnosticArray, diagnostics_callback)

    while not rospy.is_shutdown():
        rospy.sleep(0.5)

def diagnostics_callback(msg):
    rospy.loginfo("gotmsg")
    std=0.0
    gyro_status ={}
    r = TestResultRequest()
    for status in msg.status:
         if status.name == "Gyro Sensor":
             for value in status.values:
                  gyro_status[value.key]=value.value
             for el in eval(gyro_status['Calibration Buffer']):
                 std = std + (el - float(gyro_status['Calibration Offset']))**2
             std = math.sqrt(std / (len(gyro_status['Calibration Buffer'])-1))
             if std < 4.0 and (float(gyro_status['Calibration Offset']) < 560.0 or float(gyro_status['Calibration Offset']) > 440.0):
                 r.text_summary = "Calibration in bounds and std in normal range."
                 r.result = TestResultRequest.RESULT_PASS
                 r.html_result = " Calibration Offset: %s, Standard Deviation: %f"%(gyro_status['Calibration Offset'],std)
             else:
                 r.text_summary = "Calibration out of bounds or std out of normal range."
                 r.result = TestResultRequest.RESULT_FAIL
                 r.html_result = " Calibration Offset: %s, Standard Deviation: %f"%(gyro_status['Calibration Offset'],std)
             # block until the test_result service is available
             result_service = rospy.ServiceProxy('test_result', TestResult)
             rospy.wait_for_service('test_result')
             result_service.call(r)
             rospy.spin()



if __name__ == '__main__':
    test()
