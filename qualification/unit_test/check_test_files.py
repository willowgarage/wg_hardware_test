#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

PKG = 'qualification'

import roslib; roslib.load_manifest(PKG)
import rostest, unittest

from qualification.test_loader import load_tests_from_map, load_configs_from_map, load_wg_station_map
from qualification.test import Test

import os, sys

##\brief Parses launch, tests.xml and configs.xml files in qualification
class QualificationTestParser(unittest.TestCase):
    def setUp(self):
        self.test_files = {}
        self.tests_ok = load_tests_from_map(self.test_files)

        self.config_files = {}
        self.configs_ok = load_configs_from_map(self.config_files)

        self.wg_stations = {}
        self.wg_stations_ok = load_wg_station_map(self.wg_stations)

    def test_wg_station_map(self):
        self.assert_(self.wg_stations_ok, "Unable to load WG station map. Configuration file \"wg_map.xml\" is invalid")
        self.assert_(len(self.wg_stations.items()) > 0, "No WG stations loaded")

    ##\brief All test.xml files must load properly
    def test_check_tests_parsed(self):
        self.assert_(self.tests_ok, "Tests failed to load (tests.xml)")
        self.assert_(self.test_files is not None, "Tests list is None, nothing to load")
        self.assert_(len(self.test_files.items()) > 0, "No tests loaded")

        for sn, tsts in self.test_files.iteritems():
            for t in tsts:
                self.assert_(t.validate(), "Test failed to validate. Serial number: %s" % sn)

     ##\brief All config files must load successfully
    def test_check_configs_parsed(self):
        self.assert_(self.configs_ok, "Configs failed to load (tests.xml)")
        self.assert_(self.config_files is not None, "Configs list is None, nothing to load")
        self.assert_(len(self.config_files.items()) > 0, "No config scripts loaded")

        for sn, tsts in self.config_files.iteritems():
            for t in tsts:
                self.assert_(t.validate(), "Config file failed to validate. Serial number: %s" % sn)

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '-v':
        # Use to run tests verbosly
        suite = unittest.TestSuite()
        suite.addTest(QualificationTestParser('test_wg_station_map'))
        suite.addTest(QualificationTestParser('test_check_tests_parsed'))
        suite.addTest(QualificationTestParser('test_check_configs_parsed'))
        
        unittest.TextTestRunner(verbosity = 2).run(suite)
    else:
        rostest.unitrun(PKG, 'check_test_files', QualificationTestParser)


