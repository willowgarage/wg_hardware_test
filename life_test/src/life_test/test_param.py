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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#

##\author Kevin Watts
##\brief Test variables and parameters container classes

PKG = 'life_test'
import roslib
roslib.load_manifest(PKG)

import rospy

from writing_core import *


class LifeTest(object):
    """
    Holds parameters, info for each life test
    """
    def __init__(self, short = '', testid = '', name = '', desc = '', serial = '',
                 duration = 0, launch_script = '', test_type = '', power = False, params = None):
        """
        Use **kwargs in constructor. Preferred option is to initialize from XML
        """
        self._short_serial = serial
        self._id = testid
        self._name = name
        self._short = short
        self._duration = duration
        self._desc = desc
        self._launch_script = launch_script
        self._test_type = test_type
        self._power = power

        self._params = params if params else []

        self._debug_ok = False
        
        self._has_init = self._short_serial != '' and self._id != '' and self._name != '' \
            and self._launch_script != ''


    def init_xml(self, xml):
        """
        Initialize LifeTest from XML value. 
        
        Required tags:
        serial - Short value only
        name - Full name of test
        desc - Description
        script - Path to launch script
        type - Ex: 'Burn In'
        power - Needs power or not (bool)
        
        Optional:
        debug - "true" if can run outside debug mode
        duration - Default: 0

        TestParam's are initialized using nested XML nodes
        """
        self._short_serial = xml.attributes['serial'].value # Short serial only
        self._name = xml.attributes['name'].value
        self._id = xml.attributes['id'].value
        self._desc = xml.attributes['desc'].value
        self._launch_script = xml.attributes['script'].value
        self._test_type = xml.attributes['type'].value
        self._short = xml.attributes['short'].value
        self._power = xml.attributes['power'].value != 'false'

        self._debug_ok = xml.attributes.has_key('debug') and \
            xml.attributes['debug'].value.lower() == "true"
        
        if xml.attributes.has_key('duration'):
            self._duration = int(xml.attributes['duration'].value)

        params_xml = xml.getElementsByTagName('param')
        for param_xml in params_xml:
            my_param = TestParam()
            my_param.init_xml(param_xml)

            names = [ p.name for p in self._params ]
            if my_param.name in names:
                raise Exception("Param %s already exists in param list! XML: %s" % (my_param.name, xml.toprettyxml()))

            self._params.append(my_param)

        self._has_init = True
        return True
    
    @property
    def debug_ok(self): return self._debug_ok

    @property
    def need_power(self): return self._power

    @property
    def short(self):
        return self._short

    @property
    def desc(self):
        """Get description of test"""
        return self._desc

    @property
    def params(self): 
        """Get the parameters list. Read-only"""
        return self._params

    @property
    def duration(self): return self._duration

    @property
    def name(self): return self._name

    @property
    def id(self): return self._id

    @property
    def type(self): return self._test_type

    @property
    def needs_power(self):
        return self.need_power
    
    @property
    def launch_file(self):
        return self._launch_script

    def set_params(self, namespace):
        """
        Sets parameters in given namespace
        """
        for param in self._params:
            param.set_namespace(namespace)


    def get_title(self, serial):
        """
        Return a human readable title of the test/item. Used as tab name
        """
        if len(serial) == 12: # Take last few digits of SN to ID part
            return "%s %s" % (self._short, 
                                 serial[len(serial) - 3: 
                                        len(serial)])
        # Or just return the short name
        return self._short


    def validate(self):
        """
        ##\brief Called during unit testing only. Checks all files exist, are valid

        """
        if not self._has_init:
            return False

        import os, sys

        full_path = os.path.join(roslib.packages.get_pkg_dir(PKG), self._launch_script)

        if not os.path.exists(full_path):
            print >> sys.stderr, "Test %s, path %s doesn't exist" % (self._name, full_path)
            return False
        if not full_path.endswith('.launch'):
            print >> sys.stderr, "Test %s, path %s is not a launch file" % (self._name, full_path)
            return False

        return True

    def make_param_table(self):
        """
        Writes parameters to HTML table form for logging
        """
        if len(self._params) == 0:
            return '<p>No test parameters defined.</p>\n'

        html = '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += write_table_row(['Name', 'Value', 'Key', 'Description'], True)
        for param in self._params:
            html += write_table_row([param._name, param._value, param._param_name, param._desc])
        html += '</table>\n'

        return html
            
        
## Stores parameter data for each test
## Parameters are ROS parameters, and are updated in test log
## Examples: cycle rate, joint torque, roll on/off
## Allows changes in test setup or implementation to be logged automatically
class TestParam(object):
    def __init__(self):
        self._value = ''
        self._desc = ''

        self._cumulative = ''
        self._param_name = ''
        self._name = ''
        self._namespace = ''

    def init_xml(self, param_xml):
        """
        Initialize from XML

        Required elements:
        name - Name of param
        param_name - ROS name of param
        desc - Description
        value - Value of param
        rate = "true" if parameters is cumulative, or a rate
        """
        self._name = param_xml.attributes['name'].value
        self._param_name = param_xml.attributes['param_name'].value
        self._desc = param_xml.attributes['desc'].value
        self._value = param_xml.attributes['val'].value
        self._cumulative = param_xml.attributes['rate'].value == 'true'
 
    def set_namespace(self, ns):
        """
        Set parameter's value in appropriate namespace
        """
        self._namespace = ns
        rospy.set_param('/' + self._namespace + '/' + self._param_name, self._value)

    @property
    def value(self):
        try:
            val = float(self._value)
            return val
        except:
            return str(self._value)

    @property
    def name(self): return self._name

    @property
    def rate(self):
        return self._cumulative

