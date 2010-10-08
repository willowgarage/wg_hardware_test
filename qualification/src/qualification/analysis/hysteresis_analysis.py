#!/usr/bin/env python
#
# Copyright (c) 2009, Willow Garage, Inc.
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

##\author Kevin Watts
##\brief Analyzes test data from joint hysteresis tests

PKG = "qualification"
import roslib; roslib.load_manifest(PKG)

import numpy
import math

import sys
import os
import string
from time import sleep

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from StringIO import StringIO

from pr2_self_test_msgs.msg import Plot, TestParam, TestValue

class HysteresisParameters(object):
    def __init__(self):
        self.joint_name = None
        
        self.pos_effort = None
        self.neg_effort = None
        self.range_max  = None
        self.range_min  = None
        self.slope      = None
        
        self.p_gain     = None
        self.i_gain     = None
        self.d_gain     = None
        self.i_clamp    = None
        self.max_effort = None
        
        self.timeout    = None
        self.velocity   = None
        self.tolerance  = None
        self.sd_max     = None

    def validate(self):
        if self.joint_name is None: return False
        if self.pos_effort is None: return False
        if self.neg_effort is None: return False
        if self.range_max  is None: return False
        if self.range_min  is None: return False
        if self.slope      is None: return False
        if self.timeout    is None: return False
        if self.velocity   is None: return False
        if self.tolerance  is None: return False
        if self.sd_max     is None: return False

        # Don't need to validate PID gains

        return True

    def get_test_params(self):
        test_params = []
        
        test_params.append(TestParam("Joint Name", self.joint_name))
        test_params.append(TestParam("Positive Effort", str(self.neg_effort)))
        test_params.append(TestParam("Negative Effort", str(self.pos_effort)))
        test_params.append(TestParam("Max Range", str(self.range_max)))
        test_params.append(TestParam("Min Range", str(self.range_min)))
        test_params.append(TestParam("Slope", str(self.slope)))
        test_params.append(TestParam("Timeout", str(self.timeout)))
        test_params.append(TestParam("Velocity", str(self.velocity)))
        test_params.append(TestParam("Effort SD Max", str(self.sd_max)))
        test_params.append(TestParam("Effort Tolerance", str(self.tolerance)))

        # Need to write PID gains
        if self.p_gain is not None:
            test_params.append(TestParam("P Gain", str(self.p_gain)))
        if self.i_gain is not None:
            test_params.append(TestParam("I Gain", str(self.i_gain)))
        if self.d_gain is not None:
            test_params.append(TestParam("D Gain", str(self.d_gain)))
        if self.i_clamp is not None:
            test_params.append(TestParam("I Clamp", str(self.i_clamp)))

        return test_params

def get_test_value(name, value, min, max):
    return TestValue(str(name), str(value), str(min), str(max))
    
    
class HysteresisDirectionData(object):
    def __init__(self, position, effort, velocity):
        self.range_max = max(position)
        self.range_min = min(position)

        min_index = int(0.05 * len(position))
        max_index = int(0.95 * len(position))

        self.position = numpy.array(position)[min_index: max_index]
        self.effort   = numpy.array(effort)  [min_index: max_index]
        self.velocity = numpy.array(velocity)[min_index: max_index]

class HysteresisTestData(object):
    def __init__(self, positive_data, negative_data):
        self.positive = positive_data
        self.negative = negative_data

        self.range_max = max(self.positive.range_max, self.negative.range_max)
        self.range_min = min(self.positive.range_min, self.negative.range_min)

class HysteresisAnalysisResult(object):
    __slots__ = ['html', 'summary', 'result', 'values' ]
    def __init__(self):
        self.html = ''
        self.summary = ''
        self.result = False
        self.values = []

def range_analysis(params, data):
    result = HysteresisAnalysisResult()
    if (params.range_max == 0 and params.range_min == 0):
        result.summary = 'Range: Continuous.'
        result.html = '<p>Continuous joint, no range data.</p>\n'
        result.result = True
        return result

    min_obs = data.range_min 
    max_obs = data.range_max 

    fail = '<div class="error">FAIL</div>'
    ok = '<div class="pass">OK</div>'

    max_ok = True
    min_ok = True
    min_status = ok
    max_status = ok
    # Check to range against expected values
    if (min_obs > params.range_min):
      min_ok = False
      min_status = fail

    if (max_obs < params.range_max):
      max_ok = False
      max_status = fail

    result.result = min_ok and max_ok

    result.summary = 'Range: FAIL.'
    if result.result:
      result.summary = 'Range: OK.'

    if result.result:
      result.html = '<p>Range of motion: OK.</p>\n'
    elif max_ok and not min_ok:
      result.html = "<p>Mechanism did not reach expected minimum range.</p>\n"
    elif min_ok and not max_ok:
      result.html = "<p>Mechanism did not reach expected maximum range.</p>\n"
    else:
      result.html = "<p>Mechanism did not reach expected minimum or maximum range.</p>\n"

    table = ['<table border="1" cellpadding="2" cellspacing="0">\n']
    table.append('<tr><td></td><td><b>Observed</b></td><td><b>Expected</b></td><td><b>Status</b></td></tr>\n')
    table.append('<tr><td>Maximum</td><td>%.3f</td><td>%.3f</td><td>%s</td></tr>\n' % (max_obs, params.range_max, max_status))
    table.append('<tr><td>Minimum</td><td>%.3f</td><td>%.3f</td><td>%s</td></tr>\n' % (min_obs, params.range_min, min_status))
    table.append('</table>\n')

    result.html += ''.join(table)

    # Write measurements
    result.values.append(get_test_value('Max Range', max_obs, params.range_max, ''))
    result.values.append(get_test_value('Min Range', min_obs, '', params.range_min))
    
    return result

def effort_analysis(params, data):
    result = HysteresisAnalysisResult()
    
    max_avg = numpy.average(data.positive.effort)
    min_avg = numpy.average(data.negative.effort)
    max_sd = numpy.std(data.positive.effort)
    min_sd = numpy.std(data.negative.effort)
    
    effort_diff = abs(params.pos_effort - params.neg_effort)
    tolerance = params.tolerance * effort_diff
    sd_max = params.sd_max * effort_diff

    max_ok = abs(max_avg - params.pos_effort) < tolerance
    min_ok = abs(min_avg - params.neg_effort) < tolerance

    max_even = max_sd < sd_max
    min_even = min_sd < sd_max
    
    summary = 'Effort: FAIL.'
    if max_ok and min_ok and max_even and min_even:
      html = ['<p>Efforts OK.</p>\n']
      summary = 'Effort: OK.'
      result.result = True
    elif max_ok and min_ok:
      html = ['<p>Efforts have acceptable average, but are uneven.</p>\n']
    elif max_avg - params.pos_effort > tolerance and params.neg_effort - min_avg > tolerance:
      html = ['<p>Effort is too high.</p>\n']
    elif max_avg - params.pos_effort < tolerance and params.neg_effort - min_avg < tolerance:
      html = ['<p>Effort is too low.</p>\n']
    else:
      html = ['<p>Efforts are outside acceptable parameters. See graphs.</p>\n']
      
    # Standard deviations in percent of value
    sd_min_percent = abs(min_sd / effort_diff) * 100
    sd_max_percent = abs(max_sd / effort_diff) * 100

    if max_even:
      positive_sd_msg = '<div class="pass">OK</div>'
    else:
      positive_sd_msg = '<div class="warning">UNEVEN</div>'

    if min_even:
      negative_sd_msg = '<div class="pass">OK</div>'
    else:
      negative_sd_msg = '<div class="warning">UNEVEN</div>'
      
    if max_ok:
      positive_msg = '<div class="pass">OK</div>'
    else:
      positive_msg = '<div class="error">FAIL</div>'

    if min_ok:
      negative_msg = '<div class="pass">OK</div>'
    else:
      negative_msg = '<div class="error">FAIL</div>'

    html.append('<table border="1" cellpadding="2" cellspacing="0">\n')
    html.append('<tr><td><b>Name</b></td><td><b>Average</b></td><td><b>Expected</b></td><td><b>Status</b></td><td><b>SD %</b></td><td><b>Status</b></td></tr>\n')
    html.append('<tr><td>Positive</td><td>%.2f</td><td>%.2f</td><td>%s</td><td>%.2f</td><td>%s</td></tr>\n' % (max_avg, params.pos_effort, positive_msg, sd_max_percent, positive_sd_msg))
    html.append('<tr><td>Negative</td><td>%.2f</td><td>%.2f</td><td>%s</td><td>%.2f</td><td>%s</td></tr>\n' % (min_avg, params.neg_effort, negative_msg, sd_min_percent, negative_sd_msg)) 
    html.append('</table><br>\n')
    html.append('<p>Effort tolerance: %.2f. SD tolerance: %.1f percent</p>\n' % (tolerance, params.sd_max * 100))

    result.html = ''.join(html)
    result.summary = summary

    result.values = []
    result.values.append(get_test_value('Positive Effort Avg', max_avg, params.pos_effort - tolerance, params.pos_effort + tolerance))
    result.values.append(get_test_value('Positive Effort SD', max_sd, '', sd_max))
    result.values.append(get_test_value('Positive Effort SD Percent', sd_max_percent, '', params.sd_max))

    result.values.append(get_test_value('Negative Effort Avg', min_avg, params.neg_effort - tolerance, params.neg_effort + tolerance))
    result.values.append(get_test_value('Negative Effort SD', min_sd, '', sd_max))
    result.values.append(get_test_value('Positive Effort SD Percent', sd_min_percent, '', params.sd_max))

    return result

def velocity_analysis(params, data):
    html = ['<p>Search velocity: %.2f.</p><br>\n' % abs(params.velocity)]

    pos_avg = numpy.average(data.positive.velocity)
    pos_sd = numpy.std(data.positive.velocity)
    pos_rms = math.sqrt(numpy.average( (data.positive.velocity - params.velocity) * (data.positive.velocity - params.velocity) ))

    neg_avg = numpy.average(data.negative.velocity)
    neg_sd = numpy.std(data.negative.velocity)
    neg_rms = math.sqrt(numpy.average( (data.negative.velocity - params.velocity) * (data.negative.velocity - params.velocity) ))
 
    html.append('<table border="1" cellpadding="2" cellspacing="0">\n')
    html.append('<tr><td></td><td><b>Average</b></td><td><b>Std. Dev.</b></td><td><b>RMS</b></td></tr>\n')
    html.append('<tr><td>Positive</td><td>%.2f</td><td>%.3f</td><td>%.3f</td></tr>\n' % (pos_avg, pos_sd, pos_rms))
    html.append('<tr><td>Negative</td><td>%.2f</td><td>%.3f</td><td>%.3f</td></tr>\n' % (neg_avg, neg_sd, neg_rms))
    html.append('</table>\n')
    
    result = HysteresisAnalysisResult()
    result.result = True
    result.html = ''.join(html)

    result.values.append(get_test_value('Positive Velocity Avg', pos_avg, '', ''))
    result.values.append(get_test_value('Positive Velocity SD', pos_sd, '', ''))
    result.values.append(get_test_value('Positive Velocity RMS', pos_rms, '', ''))
    result.values.append(get_test_value('Negative Velocity Avg',neg_avg, '', ''))
    result.values.append(get_test_value('Negative Velocity SD', neg_sd, '', ''))
    result.values.append(get_test_value('Negative Velocity RMS', neg_rms, '', ''))

    return result

##\brief Checks slope of test data (effort v. position)
def regression_analysis(params, data):
    result = HysteresisAnalysisResult()    

    A_pos = numpy.vstack([data.positive.position, numpy.ones(len(data.positive.position))]).T
    A_neg = numpy.vstack([data.negative.position, numpy.ones(len(data.negative.position))]).T
    
    # y = ax + b
    a_max, b_max = numpy.linalg.lstsq(A_pos, data.positive.effort)[0]
    a_min, b_min = numpy.linalg.lstsq(A_neg, data.negative.effort)[0]

    # Check min/max slopes close together
    slope_avg = 0.5 * (a_max + a_min)
    slope_diff = abs((a_max - a_min)) # / slope_avg)
    slope_sd_tol = abs(params.slope * params.sd_max)

    diff_ok = slope_diff < slope_sd_tol
    pos_ok = abs(params.slope - a_max) < slope_sd_tol
    neg_ok = abs(params.slope - a_min) < slope_sd_tol

    tol_intercept = params.tolerance * (params.pos_effort - params.neg_effort)
    pos_int_ok = abs(b_max - params.pos_effort) < tol_intercept
    neg_int_ok = abs(b_min - params.neg_effort) < tol_intercept

    slope_ok = diff_ok and pos_ok and neg_ok and pos_int_ok and neg_int_ok

    ok_dict = {True: 'OK', False: 'FAIL'}

    if slope_ok:
      result.summary = "Effort: OK"
      result.result = True
    else:
      result.summary = "Effort: FAIL"
    
    ##\todo Check if slope is significant....
    html = ['<p>Simple linear regression on the effort and position plot data. See tables below for slopes, intercepts of effort plot. Both the positive and negative slopes must be in tolerance, and the difference between slopes must be acceptable.</p>\n']
    html.append('<table border="1" cellpadding="2" cellspacing="0">\n')
    html.append('<tr><td><b>Name</b></td><td><b>Slope</b></td><td><b>Expected</b><td><b>Tolerance</b></td><td><b>Passed</b></td>\n')
    html.append('<tr><td>Positive</td><td>%.3f</td><td>%.3f</td><td>%.3f</td><td>%s</td></tr>\n' % (a_max, params.slope, slope_sd_tol, ok_dict[pos_ok]))
    html.append('<tr><td>Negative</td><td>%.3f</td><td>%.3f</td><td>%.3f</td><td>%s</td></tr>\n' % (a_min, params.slope, slope_sd_tol, ok_dict[neg_ok]))
    html.append('<tr><td>Difference</td><td>%.3f</td><td>%.3f</td><td>%.3f</td><td>%s</td></tr>\n' % (a_max - a_min, 0, slope_sd_tol, ok_dict[diff_ok]))
    html.append('</table>\n')

    html.append('<p>Slope intercepts must be within tolerance of expected values, or the average effort is incorrect.</p>\n')
    html.append('<table border="1" cellpadding="2" cellspacing="0">\n')
    html.append('<tr><td><b>Name</b></td><td><b>Intercept</b></td><td><b>Expected</b><td><b>Tolerance</b></td><td><b>Passed</b></td>\n')
    html.append('<tr><td>Positive</td><td>%.3f</td><td>%.3f</td><td>%.3f</td><td>%s</td></tr>\n' % (b_max, params.pos_effort, tol_intercept, ok_dict[pos_int_ok]))
    html.append('<tr><td>Negative</td><td>%.3f</td><td>%.3f</td><td>%.3f</td><td>%s</td></tr>\n' % (b_min, params.neg_effort, tol_intercept, ok_dict[neg_int_ok]))
    html.append('</table>\n')
    
    result.html = ''.join(html)

    result.values.append(get_test_value('Slope Positive', a_max, params.slope - slope_sd_tol, params.slope + slope_sd_tol))
    result.values.append(get_test_value('Slope Negative', a_min, params.slope - slope_sd_tol, params.slope + slope_sd_tol))
    result.values.append(get_test_value('Intercept Positive', b_max, params.pos_effort - tol_intercept, params.pos_effort + tol_intercept))
    result.values.append(get_test_value('Intercept Negative', b_min, params.neg_effort - tol_intercept, params.neg_effort + tol_intercept))

    return result

def plot_effort(params, data):
    # Plot the analyzed data
    fig = plt.figure(1)
    axes1 = fig.add_subplot(211)
    axes2 = fig.add_subplot(212)
    axes2.set_xlabel('Position (Radians or Meters)')
    axes1.set_ylabel('Effort (Nm or N)')
    axes2.set_ylabel('Effort (Nm or N)')
    axes1.plot(data.positive.position, data.positive.effort, 'b--', label='_nolegend_')
    axes2.plot(data.negative.position, data.negative.effort, 'b--', label='_nolegend_')

    max_avg = numpy.average(data.positive.effort)
    min_avg = numpy.average(data.negative.effort)
    max_sd = numpy.std(data.positive.effort)
    min_sd = numpy.std(data.negative.effort)

    axes1.axhline(y = max_avg, color = 'r', label='Average')
    axes1.axhline(y = max_avg + max_sd, color = 'y', label='Error bars')
    axes1.axhline(y = max_avg - max_sd, color = 'y', label='_nolegend_')
    
    axes2.axhline(y = min_avg, color = 'r', label='Average')
    axes2.axhline(y = min_avg - min_sd, color = 'y', label='Error bars')
    axes2.axhline(y = min_avg + min_sd, color = 'y', label='_nolegend_')
            
    # Add expected efforts to both plots
    axes1.axhline(y = params.pos_effort, color = 'g', label='Expected')
    axes2.axhline(y = params.neg_effort, color = 'g', label='Expected')
    
    fig.text(.3, .95, params.joint_name + ' Hysteresis Efforts')
    #axes1.legend(shadow=True)

    stream = StringIO()
    plt.savefig(stream, format = "png")
    image = stream.getvalue()
    plt.close()
    
    p = Plot()
    p.title = params.joint_name + "_hysteresis1"
    p.image = image
    p.image_format = "png"

    return p

def plot_velocity(params, data):
    fig = plt.figure(2)
    plt.ylabel('Velocity')
    plt.xlabel('Position')
    plt.plot(numpy.array(data.positive.position), numpy.array(data.positive.velocity), 'b--', label='Data')
    plt.plot(numpy.array(data.negative.position), numpy.array(data.negative.velocity), 'b--', label='_nolegened_')
    plt.axhline(y = params.velocity, color = 'g', label = 'Command')
    plt.axhline(y = -1 * params.velocity, color = 'g', label = '_nolegend_')
    
    fig.text(.3, .95, params.joint_name + ' Hysteresis Velocity')
    plt.legend(shadow=True)
    
    stream = StringIO()
    plt.savefig(stream, format = "png")
    image = stream.getvalue()
    
    p = Plot()
    p.title = params.joint_name + "_hysteresis2"
    p.image = image
    p.image_format = "png"
    
    plt.close()
    
    return p

# Wrist analysis starts here

class WristRollHysteresisTestData(HysteresisTestData):
    def __init__(self, msg):
        left_min = int(0.05 * len(msg.left_turn.roll_position))
        left_max = int(0.95 * len(msg.left_turn.roll_position))
        right_min = int(0.05 * len(msg.right_turn.roll_position))
        right_max = int(0.95 * len(msg.right_turn.roll_position))

        self.pos_flex_effort = numpy.array(msg.left_turn.flex_effort) [left_min: left_max]
        self.neg_flex_effort = numpy.array(msg.right_turn.flex_effort)[right_min: right_max]

        self.positive = HysteresisDirectionData(msg.left_turn.roll_position, msg.left_turn.roll_effort, msg.left_turn.roll_velocity)
        self.negative = HysteresisDirectionData(msg.right_turn.roll_position, msg.right_turn.roll_effort, msg.right_turn.roll_velocity)

class WristRollHysteresisParams(HysteresisParameters):
    def __init__(self, msg):
        self.joint_name  = msg.roll_joint
        self.p_gain      = msg.roll_pid[0]
        self.i_gain      = msg.roll_pid[1]
        self.d_gain      = msg.roll_pid[2]
        self.i_clamp     = msg.roll_pid[3]

        self.velocity    = msg.arg_value[1]
        self.tolerance   = msg.arg_value[2]
        self.sd_max      = msg.arg_value[3]

        self.timeout     = msg.arg_value[4]
        self.pos_effort  = msg.arg_value[5]
        self.neg_effort  = msg.arg_value[6]

        self.range_max   = 0
        self.range_min   = 0
        self.slope       = 0

        # Subclass params only
        self.flex_joint   = msg.flex_joint
        self.flex_tol     = msg.arg_value[7]
        self.flex_max     = msg.arg_value[8]
        self.flex_sd      = msg.arg_value[9]
        self.flex_p_gain  = msg.flex_pid[0]
        self.flex_i_gain  = msg.flex_pid[1]
        self.flex_d_gain  = msg.flex_pid[2]
        self.flex_i_clamp = msg.flex_pid[3]

    def get_test_params(self):
        test_params = HysteresisParameters.get_test_params(self)

        test_params.append(TestParam("Flex Joint", self.flex_joint))
        test_params.append(TestParam("Flex Tolerance", str(self.flex_tol)))
        test_params.append(TestParam("Flex Max", str(self.flex_max)))
        test_params.append(TestParam("Flex SD", str(self.flex_sd)))
        test_params.append(TestParam("Flex P Gain", str(self.flex_p_gain)))
        test_params.append(TestParam("Flex I Gain", str(self.flex_i_gain)))
        test_params.append(TestParam("Flex D Gain", str(self.flex_d_gain)))
        test_params.append(TestParam("Flex I Clamp", str(self.flex_i_clamp)))

        return test_params

##\brief Analyzes flex effort during wrist difference test
def wrist_flex_analysis(params, data):
    flex_max = numpy.average(data.pos_flex_effort)
    flex_max_sd = numpy.std(data.pos_flex_effort)
    flex_min = numpy.average(data.neg_flex_effort)
    flex_min_sd = numpy.std(data.neg_flex_effort)
    flex_max_val = max(numpy.max(data.pos_flex_effort), numpy.max(data.neg_flex_effort))
    flex_min_val = min(numpy.min(data.pos_flex_effort), numpy.min(data.neg_flex_effort))
    
    max_msg = '<div class="pass">OK</div>'
    if abs(flex_max) > params.flex_tol:
        max_msg = '<div class="error">FAIL</div>'
        
    min_msg = '<div class="pass">OK</div>'
    if abs(flex_min) > params.flex_tol:
        min_msg = '<div class="error">FAIL</div>'
        
    max_val_msg = '<div class="pass">OK</div>'
    if abs(flex_max_val) > params.flex_max:
        max_val_msg = '<div class="error">FAIL</div>'

    min_val_msg = '<div class="pass">OK</div>'
    if abs(flex_min_val) > params.flex_max:
        min_val_msg = '<div class="error">FAIL</div>'

    max_sd_msg = '<div class="pass">OK</div>'
    if abs(flex_max_sd) > params.flex_sd:
        max_sd_msg = '<div class="error">FAIL</div>'
        
    min_sd_msg = '<div class="pass">OK</div>'
    if abs(flex_min_sd) > params.flex_sd:
        min_sd_msg = '<div class="error">FAIL</div>'
        
    ok = abs(flex_max) < params.flex_tol and abs(flex_min) < params.flex_tol
    ok = ok and abs(flex_min_val) < params.flex_max and abs(flex_max_val) < params.flex_max
    
    if ok:
        html = ['<p>Wrist flex effort is OK.</p>']
    else:
        html = ['<p>Wrist flex effort failed. Check graphs.</p>']
        
    html.append('<p>Flex effort maximum values. Allowed maximum: %.2f</p>' % (params.flex_max))
    html.append('<table border="1" cellpadding="2" cellspacing="0">')
    html.append('<tr><td></td><td><b>Effort</b></td><td><b>Status</b></td></tr>')
    html.append('<tr><td><b>Max Value</b></td><td>%.2f</td><td>%s</td></tr>' % (flex_max_val, max_val_msg))
    html.append('<tr><td><b>Min Value</b></td><td>%.2f</td><td>%s</td></tr>' % (flex_min_val, min_val_msg))
    html.append('</table>')
    
    html.append('<br>')
    html.append('<p>Flex effort average and noise, both directions. Allowed absolute average: %.2f. Allowed noise: %.2f</p>' % (params.flex_tol, params.flex_sd))
    html.append('<table border="1" cellpadding="2" cellspacing="0">')
    html.append('<tr><td><b>Effort Avg</b></td><td><b>Status</b></td><td><b>Effort SD</b></td><td><b>SD Status</b></td></tr>')
    html.append('<tr><td>%.2f</td><td>%s</td><td>%.2f</td><td>%s</td></tr>' % (flex_max, max_msg, flex_max_sd, max_sd_msg))
    html.append('<tr><td>%.2f</td><td>%s</td><td>%.2f</td><td>%s</td></tr>' % (flex_min, min_msg, flex_min_sd, min_sd_msg))
    html.append('</table>')
    
    result = HysteresisAnalysisResult()
    result.html = '\n'.join(html)
    result.result = ok
    
    result.values = [
        get_test_value('Flex Pos Average', flex_max, '', params.flex_tol),
        get_test_value('Flex Pos SD', flex_max_sd, '', params.flex_sd),
        get_test_value('Flex Neg Average', flex_min, '', params.flex_tol),
        get_test_value('Flex Neg SD', flex_min_sd, '', params.flex_sd),
        get_test_value('Flex Max Val', flex_max_val, '', params.flex_max),
        get_test_value('Flex Min Val', flex_min_val, '', -1 * params.flex_max) 
        ]

    return result

##\brief Plots effort of wrist flex during qual
def plot_flex_effort(params, data):
    # Can move to separate plotting function
    # Plot the analyzed data
    flex_max_tol = params.flex_max
    
    fig = plt.figure(1)
    axes1 = fig.add_subplot(211)
    axes2 = fig.add_subplot(212)
    axes2.set_xlabel('Roll Position')
    axes1.set_ylabel('Effort (+ dir)')
    axes2.set_ylabel('Effort (- dir)')
    
    axes1.plot(data.positive.position, data.pos_flex_effort, 'b--', label='_nolegend_')
    axes1.axhline(y = flex_max_tol, color = 'y', label='Max Value')
    axes1.axhline(y = - flex_max_tol, color = 'y', label='_nolegend_')
    axes1.axhline(y = 0, color = 'r', label='_nolegend_')
    
    axes2.plot(data.negative.position, data.neg_flex_effort, 'b--', label='_nolegend_')
    axes2.axhline(y = flex_max_tol, color = 'y', label='Max Value')
    axes2.axhline(y = - flex_max_tol, color = 'y', label='_nolegend_')
    axes2.axhline(y = 0, color = 'r', label='_nolegend_')
    
    fig.text(.4, .95, 'Wrist Flex Effort')
    
    stream = StringIO()
    plt.savefig(stream, format = "png")
    image = stream.getvalue()
    
    p = Plot()
    p.title = "wrist_diff_flex_effort"
    p.image = image
    p.image_format = "png"
    
    plt.close()
    
    return p

def make_wrist_test_summary(roll_stat, flex_stat):
    if flex_stat and roll_stat:
        return "Wrist Difference check OK. Motors are symmetric."
    if flex_stat and not roll_stat:
        return "Wrist roll hysteresis failed. Wrist flex effort OK."
    if not flex_stat and roll_stat:
        return "Wrist roll hysteresis OK. Wrist flex effort failed."
    return "Wrist roll hystereis and flex effort failed."

##\brief Checks wrist data before analysis
def wrist_hysteresis_data_present(data):
    return data.positive.position.size > 100 and data.negative.position.size > 100
