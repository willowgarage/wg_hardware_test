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

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

import os, sys
from xml.dom import minidom

from test import Test
from wg_station import WGTestStation

TESTS_DIR = os.path.join(roslib.packages.get_pkg_dir(PKG), 'tests')
CONFIG_DIR = os.path.join(roslib.packages.get_pkg_dir(PKG), 'config')
QUAL_DIR = roslib.packages.get_pkg_dir(PKG)

def load_tests_from_map(tests, debugs = []):
  """
  Loads tests from tests/tests.xml configuration file

  \param tests {} : Test by serial. { str: [ Test ] }
  \return True if loaded successfully and all tests validated
  """
  # Load test directory
  tests_xml_path = os.path.join(TESTS_DIR, 'tests.xml')
  try:
    doc = minidom.parse(tests_xml_path)
  except IOError:
    print >> sys.stderr, "Could not load tests description from '%s'. IOError, file may be invalid."%(tests_xml_path)
    return False  
  except Exception:
    print >> sys.stderr, "Could not load tests description from '%s'. Unknown exception."%(tests_xml_path)
    import traceback
    traceback.print_exc()
    return False  
  
  # Loads tests by serial number of part
  test_elements = doc.getElementsByTagName('test')
  for tst in test_elements:
    if not tst.attributes.has_key('serial'):
      print >> sys.stderr, "Tst XML element does not have attribute \"serial\". Unable to load. XML: %s" % str(tst)
      return False
    serial = tst.attributes['serial'].value
    if not len(serial) == 7: 
      print >> sys.stderr, "Serial number is invalid: %s" % serial
      return False

    if not tst.attributes.has_key('file'):
      print >> sys.stderr, "Test XML element does not have attribute \"file\". Unable to load. XML: %s" % str(tst)
      return False
    test_file = tst.attributes['file'].value

    if not tst.attributes.has_key('descrip'):
      print >> sys.stderr, "Test XML element does not have attribute \"descrip\". Unable to load. XML: %s" % str(tst)
      return False
    descrip = tst.attributes['descrip'].value

    # Mark as debug test, which allows us to run outside debug mode
    debug_test = tst.attributes.has_key('debug') and tst.attributes['debug'].value.lower() == "true"
    if debug_test:
      debugs.append(serial)

    test_path = os.path.join(os.path.join(TESTS_DIR, test_file))
    test_dir = os.path.dirname(test_path)
    test_str = open(test_path).read()

    my_test = Test(descrip, serial[3:7])
    try:
      if not my_test.load(test_str, test_dir):
        print >> sys.stderr, "Unable to load test %s" % descrip
        return False
    except Exception, e:
      print >> sys.stderr, "Unable to load test %s" % descrip
      import traceback
      traceback.print_exc()
      return False

    my_test.debug_ok = debug_test

    tests.setdefault(serial, []).append(my_test)
    
  return True

def load_configs_from_map(config_files):
  """
  Loads configuration scripts from config/configs.xml configuration file

  \param tests {} : Configs by serial. { str: [ Test ] }
  \return True if loaded successfully and all tests validated
  """
  # Load part configuration scripts
  config_xml_path = os.path.join(CONFIG_DIR, 'configs.xml')
  try:
    doc = minidom.parse(config_xml_path)
  except IOError:
    print >> sys.stderr, "Could not load configuation scripts from '%s'. IOError, file may be invalid."%(config_xml_path)
    return False
  except Exception:
    print >> sys.stderr, "Could not load tests description from '%s'. Unknown exception."%(tests_xml_path)
    import traceback
    traceback.print_exc()
    return False  
    
  config_elements = doc.getElementsByTagName('config')
  for conf in config_elements:
    if not conf.attributes.has_key('serial'):
      print >> sys.stderr, "Test XML element does not have attribute \"serial\". Unable to load. XML: %s" % str(conf)
      return False
    serial = conf.attributes['serial'].value
    if not len(serial) == 7: 
      print >> sys.stderr, "Serial number is invalid: %s" % serial
      return False

    if not conf.attributes.has_key('file'):
      print >> sys.stderr, "Test XML element does not have attribute \"file\". Unable to load. XML: %s" % str(conf)
      return False
    test_file = conf.attributes['file'].value

    if not conf.attributes.has_key('descrip'):
      print >> sys.stderr, "Test XML element does not have attribute \"descrip\". Unable to load. XML: %s" % str(conf)
      return False
    descrip = conf.attributes['descrip'].value
      
    powerboard = True
    if conf.attributes.has_key('powerboard'):
      powerboard = conf.attributes['powerboard'].value.lower() == "true"
      
    timeout = 600
    if conf.attributes.has_key('timeout'):
      timeout = int(conf.attributes['timeout'].value)

    # Generate test XML. If we need power board, add prestartup/shutdown
    # to turn on/off power
    tst = ['<test name="%s" id="%s" >' % (descrip, serial)]
    if powerboard:
      tst.append('<pre_startup name="Power On" timeout="30">scripts/power_cycle.launch</pre_startup>')
    tst.append('<pre_startup name="%s" timeout="%d">config/%s</pre_startup>' % (descrip, timeout, test_file))
    tst.append('<subtest name="%s Test" timeout="30">config/subtest_conf.launch</subtest>' % (descrip))
    if powerboard:
      tst.append('<shutdown name="Shutdown" timeout="30">scripts/power_board_disable.launch</shutdown>')
    tst.append('</test>')

    test_str = '\n'.join(tst)

    my_conf = Test(descrip, serial[3:7])
    if not my_conf.load(test_str, QUAL_DIR):
      print >> sys.stderr, "Unable to load test %s" % descrip
      print >> sys.stderr, "Test XML: %s" % tst
      return False
  
    my_conf.debug_ok = conf.attributes.has_key('debug') and conf.attributes['debug'].value.lower() == "true"
  
    config_files.setdefault(serial, []).append(my_conf)

  return True


def load_wg_station_map(wg_teststations):
  """
  Loads "map" of WG test stations. Each station has a GUI, a remote host
  and a power board.
  \param wg_teststations { str : WGTestStation } : Output dictionary of test stations
  \return True if loaded successfully
  """
  map_xml_path = os.path.join(roslib.packages.get_pkg_dir(PKG), 'wg_map.xml')
  
  try:
    doc = minidom.parse(map_xml_path)
  except IOError:
    print >> sys.stderr, "Could not load test map from '%s'"%(map_xml_path)
    return False

  stations = doc.getElementsByTagName('station')

  for st in stations:
    my_station = WGTestStation()

    if not my_station.xmlLoad(st):
      return False
    wg_teststations[my_station.gui_host] = my_station

  return True
