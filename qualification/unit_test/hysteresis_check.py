#!/usr/bin/env python

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

import qualification.analysis.hysteresis_analysis as hyst_analysis

import rostest, unittest

import math, numpy
import sys, os
import copy

import random
random.seed()

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
    

        
if __name__ == '__main__':
    rostest.unitrun(PKG, 'check_hysteresis', TestHysteresis)


