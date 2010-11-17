#!/usr/bin/env python
#
# Copyright (c) 2010, Willow Garage, Inc.
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

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

import qualification.analysis.hysteresis_analysis as hyst_analysis

import rostest, unittest

import math, numpy
import sys, os
import copy

import random
random.seed()

class DummyWristRollHysteresisTestData(hyst_analysis.HysteresisTestData): 
    def __init__(self, pos_data, neg_data):
        hyst_analysis.HysteresisTestData.__init__(self, pos_data, neg_data)

        self.pos_flex_effort = numpy.array([ 0.0 for x in pos_data.position ])
        self.neg_flex_effort = numpy.array([ 0.0 for x in neg_data.position ])
                

class TestHysteresis(unittest.TestCase):
    def setUp(self):
        self.pos_position = [ 0.001 * i for i in range(-1100, 1100)]
        self.neg_position = [-0.001 * i for i in range(-1100, 1100)]

        self.pos_velocity = [ 0.5 for i in range(-1100, 1100)]
        self.neg_velocity = [-0.5 for i in range(-1100, 1100)]

        self.pos_effort = [ 1.0 for i in range(-1100, 1100)]
        self.neg_effort = [-1.0 for i in range(-1100, 1100)]

        # Make some data
        self.pos_data = hyst_analysis.HysteresisDirectionData(self.pos_position, self.pos_effort, self.pos_velocity)
        self.neg_data = hyst_analysis.HysteresisDirectionData(self.neg_position, self.neg_effort, self.neg_velocity)

        self.data = hyst_analysis.HysteresisTestData(self.pos_data, self.neg_data)
        
        # Set some parameters
        self.params = hyst_analysis.HysteresisParameters()
        self.params.pos_effort =  1.0
        self.params.neg_effort = -1.0
        self.params.range_max  =  1.0
        self.params.range_min  = -1.0
        self.params.velocity   =  0.5
        self.params.tolerance  =  0.1
        self.params.sd_max     =  0.2
        self.params.slope      =  0.0
        self.params.joint_name = "dummy_jnt"

        # Wrist data is a little different
        self.wrist_roll_data = DummyWristRollHysteresisTestData(self.pos_data, self.neg_data)

        # Wrist params also a bit different
        self.wrist_roll_params = copy.deepcopy(self.params)
        self.wrist_roll_params.flex_joint   = 'wrist_flx_jnt'
        self.wrist_roll_params.flex_tol     = 0.5
        self.wrist_roll_params.flex_max     = 1.0
        self.wrist_roll_params.flex_sd      = 0.25
        self.wrist_roll_params.flex_p_gain  = 1.0
        self.wrist_roll_params.flex_i_gain  = 1.0
        self.wrist_roll_params.flex_d_gain  = 1.0
        self.wrist_roll_params.flex_i_clamp = 1.0

    def test_range(self):
        # Our current data should pass
        good = hyst_analysis.range_analysis(self.params, self.data)
        self.assert_(good.result, "Hystersis range result didn't pass: %s\n%s" % (good.summary, good.html))

        # Modify the params to move outside the max range
        bad_params = copy.deepcopy(self.params)
        bad_params.range_max = max(self.pos_position) + 0.1
        bad_params.range_min = min(self.pos_position) - 0.1
        
        bad = hyst_analysis.range_analysis(bad_params, self.data)
        self.assert_(not bad.result, "Hysteresis didn't fail for invalid range. Message: %s\n%s" % (bad.summary, bad.html))

        # Check range data for continuous joint
        cont_range = copy.deepcopy(self.params)
        cont_range.range_max = 0.0
        cont_range.range_min = 0.0
        
        cont = hyst_analysis.range_analysis(cont_range, self.data)
        self.assert_(cont.result, "Continuous result failed! Summary: %s" % cont.summary)
        self.assert_(cont.summary.lower().find('continuous') > -1, "Continuous check failed to pick up that it was continuous. Summary: %s\n%s" % (cont.summary, cont.html))


    def test_effort(self):
        # Our current data should pass
        good = hyst_analysis.range_analysis(self.params, self.data)
        self.assert_(good.result, "Hystersis effort result didn't pass: %s" % good.summary)

        # Add effort to data
        pos_high_effort = [ x + 0.5 for x in self.pos_effort ]
        neg_high_effort = [ x - 0.5 for x in self.neg_effort ]
        pos_high_data = hyst_analysis.HysteresisDirectionData(self.pos_position, pos_high_effort, self.pos_velocity)
        neg_high_data = hyst_analysis.HysteresisDirectionData(self.neg_position, neg_high_effort, self.neg_velocity)

        high_effort_data = hyst_analysis.HysteresisTestData(pos_high_data, neg_high_data)

        high = hyst_analysis.effort_analysis(self.params, high_effort_data)
        self.assert_(not high.result, "Effort didn't fail for high effort! %s\n%s" % (high.summary, high.html))

        # Add noise to data
        pos_uneven_effort = [ random.gauss(x, 0.5 * x) for x in self.pos_effort ]
        neg_uneven_effort = [ random.gauss(x, 0.5 * x) for x in self.neg_effort ]

        # Make sure that we're actually noisy. This is random noise, after all
        noisy = numpy.std(pos_uneven_effort) > self.params.sd_max or numpy.std(neg_uneven_effort) > self.params.sd_max 
        
        pos_uneven_data = hyst_analysis.HysteresisDirectionData(self.pos_position, pos_uneven_effort, self.pos_velocity)
        neg_uneven_data = hyst_analysis.HysteresisDirectionData(self.neg_position, neg_uneven_effort, self.neg_velocity)

        uneven_effort_data = hyst_analysis.HysteresisTestData(pos_uneven_data, neg_uneven_data)

        uneven = hyst_analysis.effort_analysis(self.params, uneven_effort_data)
        self.assert_(noisy and not uneven.result, "Effort didn't fail for uneven effort, but was noisy! %s\n%s" % (uneven.summary, uneven.html))
        

    def test_velocity(self):
        # Just a smoke test...
        rv = hyst_analysis.velocity_analysis(self.params, self.data)
        self.assert_(rv.result, "Velocity failed: %s\n%s" % (rv.summary, rv.html))


    def test_slope(self):
        # Simulate some slope data, make sure we're OK
        slope_params = copy.deepcopy(self.params)
        slope_params.slope      =  1.0
        slope_params.pos_effort =  1.0
        slope_params.neg_effort = -1.0        

        pos_slope_effort = [ x * slope_params.slope + slope_params.pos_effort for x in self.pos_position ]
        neg_slope_effort = [ x * slope_params.slope + slope_params.neg_effort for x in self.neg_position ]

        
        pos_slope_data = hyst_analysis.HysteresisDirectionData(self.pos_position, pos_slope_effort, self.pos_velocity)
        neg_slope_data = hyst_analysis.HysteresisDirectionData(self.neg_position, neg_slope_effort, self.neg_velocity)


        slope_data = hyst_analysis.HysteresisTestData(pos_slope_data, neg_slope_data)
        
        rv = hyst_analysis.regression_analysis(slope_params, slope_data)
        self.assert_(rv.result, "Slope check failed: %s\n%s" % (rv.summary, rv.html))
    
    def test_plots(self):
        # Smoke tests on plot functions
        p_eff = hyst_analysis.plot_effort(self.params, self.data)
        p_vel = hyst_analysis.plot_velocity(self.params, self.data)
    
    def test_wrist_analysis(self):
        # Check that our data is valid
        self.assert_(hyst_analysis.wrist_hysteresis_data_present(self.wrist_roll_data), "Unable to confirm that wrist data was valid. Should have valid data")

        # Check wrist flex analysis.
        flex_analysis = hyst_analysis.wrist_flex_analysis(self.wrist_roll_params, self.wrist_roll_data)
        self.assert_(flex_analysis.result, "Wrist flex analysis failed! %s\n%s" % (flex_analysis.summary,
                                                                                   flex_analysis.html))

        # Smoke test on the plots
        p = hyst_analysis.plot_flex_effort(self.wrist_roll_params, self.wrist_roll_data)
        
        
if __name__ == '__main__':
    rostest.unitrun(PKG, 'check_hysteresis', TestHysteresis)


