#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

import sys, os

class WGTestStation(object):
    """
    Container class for WG test station configuration to operate qual system
    """
    def __init__(self):
        self._power_board = None
        self._breaker0 = False
        self._breaker1 = False
        self._breaker2 = False

        self._gui_host = None
        self._test_host = None

        self._envs = {}

    def set_envs(self):
        """
        Set any environment variables specific to machine.
        """
        for k, v in self._envs.iteritems():
            os.environ[k] = v

    # Machine
    @property
    def gui_host(self): return self._gui_host
    @property
    def test_host(self): return self._test_host

    # Power system
    @property
    def powerboard(self): 
        if self._power_board:
            return self._power_board
        return '0000'
    @property
    def breaker0(self): return self._breaker0
    @property
    def breaker1(self): return self._breaker1
    @property
    def breaker2(self): return self._breaker2

    def xmlLoad(self, xmlDoc):
        """
        Load machine configuration from XML. 
        Required parameters: 
          "gui" - GUI machine. Must be full machine name
          "host" - Remote host name
        Optional:
          "powerboard" - SN of power board
          "breaker[0-2]" - Bool, to enable power breaker
          "env" - Tag <env name="FOO" value="bar" />

        @param xmlDoc XML : 
        """
        if not xmlDoc.attributes.has_key('gui'):
            print >> sys.stderr, "Unable to find attribute \"gui\" in XML doc for test station. XML: %s" % str(xmlDoc)
            return False
        self._gui_host = xmlDoc.attributes['gui'].value

        if not xmlDoc.attributes.has_key('host'):
            print >> sys.stderr, "Unable to find attribute \"host\" in XML doc for test station. XML: %s" % str(xmlDoc)
            return False
        self._test_host = xmlDoc.attributes['host'].value

        if xmlDoc.attributes.has_key('powerboard'):
            self._power_board = xmlDoc.attributes['powerboard'].value
            if not len(self._power_board) == 4 and unicode(self._power_board).isnumeric():
                print >> sys.stderr, "Power board entry is invalid. Must be four digits. Ex: \"1001\""
                return False

        if self._power_board and xmlDoc.attributes.has_key('breaker0'):
            self._breaker0 = xmlDoc.attributes['breaker0'].value.lower() == 'true'

        if self._power_board and xmlDoc.attributes.has_key('breaker1'):
            self._breaker1 = xmlDoc.attributes['breaker1'].value.lower() == 'true'

        if self._power_board and xmlDoc.attributes.has_key('breaker2'):
            self._breaker2 = xmlDoc.attributes['breaker2'].value.lower() == 'true'

        envs_xml = xmlDoc.getElementsByTagName('env')
        for env_xml in envs_xml:
            name = env_xml.attributes['name'].value
            value = env_xml.attributs['value'].value

            self._envs[name] = value

        return True

