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
##\brief Container class for WG test station configuration to operate qual system

import sys

class WGTestStation(object):
    def __init__(self):
        self._power_board = '0000'
        self._breaker0 = False
        self._breaker1 = False
        self._breaker2 = False

        self._gui_host = None
        self._test_host = None

    # Machine
    @property
    def gui_host(self): return self._gui_host
    @property
    def test_host(self): return self._test_host

    # Power system
    @property
    def powerboard(self): return self._power_board
    @property
    def breaker0(self): return self._breaker0
    @property
    def breaker1(self): return self._breaker1
    @property
    def breaker2(self): return self._breaker2

    def xmlLoad(self, xmlDoc):
        if not xmlDoc.attributes.has_key('gui'):
            print >> sys.stderr, "Unable to find attribute \"gui\" in XML doc for test station. XML: %s" % str(xmlDoc)
            return False
        self._gui_host = xmlDoc.attributes['gui'].value

        if not xmlDoc.attributes.has_key('host'):
            print >> sys.stderr, "Unable to find attribute \"host\" in XML doc for test station. XML: %s" % str(xmlDoc)
            return False
        self._test_host = xmlDoc.attributes['host'].value

        if not xmlDoc.attributes.has_key('powerboard'):
            print >> sys.stderr, "Unable to find attribute \"powerboard\" in XML doc for test station. XML: %s" % str(xmlDoc)
            return False
        self._power_board = xmlDoc.attributes['powerboard'].value

        if not xmlDoc.attributes.has_key('breaker0'):
            print >> sys.stderr, "Unable to find attribute \"breaker0\" in XML doc for test station. XML: %s" % str(xmlDoc)
            return False
        self._breaker0 = xmlDoc.attributes['powerboard'].value.lower() == 'true'

        if not xmlDoc.attributes.has_key('breaker1'):
            print >> sys.stderr, "Unable to find attribute \"breaker1\" in XML doc for test station. XML: %s" % str(xmlDoc)
            return False
        self._breaker1 = xmlDoc.attributes['powerboard'].value.lower() == 'true'

        if not xmlDoc.attributes.has_key('breaker2'):
            print >> sys.stderr, "Unable to find attribute \"breaker2\" in XML doc for test station. XML: %s" % str(xmlDoc)
            return False
        self._breaker2 = xmlDoc.attributes['powerboard'].value.lower() == 'true'

        return True

