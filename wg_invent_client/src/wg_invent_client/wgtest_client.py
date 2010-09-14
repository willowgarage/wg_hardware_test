#! /usr/bin/env python
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

"""
usage: %(progname)s [args]
"""

import os, sys, string, time, getopt

import urllib, urllib2
import invent_client
import neo_cgi, neo_util

import yaml
import types

import simple_hdfhelp as hdfhelp
import attachment_help

def obj2hdf(prefix, obj, hdf=None):
  """
  Converts python object to HDF. Not loop-safe
  """
  return py2hdf(prefix, obj.__dict__)

def py2hdf(k, v, hdf=None):
  """
  Not loop-safe
  """
  if not hdf: hdf = neo_util.HDF()

  k.replace(' ', '_')

  if type(v) == str:
    if len(v) == 0:
      v = '-'
    elif v == 'None':
      v = '-' # None values are invalid

    hdf.setValue(k, v)
  elif type(v) == types.NoneType:
    pass
  elif type(v) == unicode:
    hdf.setValue(k, str(v))
  elif type(v) in (int, float, long):
    hdf.setValue(k, str(v))
  elif type(v) in (list, tuple):
    n = 0
    for item in v:
      n = n + 1
      py2hdf("%s.%d" % (k,n), item, hdf)
  elif type(v) == dict:
    n = 0
    for _k,_v in sorted(v.iteritems()):
      if not type(_k) == str:
        _k = str(_k)
      _k = _k.replace(" ", "_")
      py2hdf(k + "." + _k, _v, hdf)
  elif type(v) == types.InstanceType or type(v) == types.ClassType \
        or isinstance(v, object):
    py2hdf(k, v.__dict__, hdf)
  
  return hdf

def _clean_val(val):
  if not type(val) == str:
    val = str(val)

  return val.replace(' ', '_').replace('.', '').replace('-', '_')

class Value(object):
  """
  Stores measurement value
  """
  def __init__(self, value, min, max):
    self.value = value
    self.min = min
    self.max = max


class SubtestData(object):
  """
  Stores subtest data to log into WG Test database
  """
  def __init__(self, name, result):
    """
    @param name str : Name of subtest
    @param result str : Result of subtest (ex: 'Pass')
    """
    self.name = name
    self.result = result

    self.note = ''
    self.parameters = {}
    self.measurements = {}

  def set_parameter(self, key, value):
    """
    Parameters are specific inputs to each subtest
    @param key str : Parameter name
    @param value str : Parameter value
    """
    k = _clean_val(key)
    self.parameters[k] = value

  def set_measurement(self, key, value, min, max):
    """
    Measurements store specific quantities that are measured for each subtest
    @param key str : Name of measurment
    @param value str : Measurement value
    @param min str : Minimum value
    @param max str : Maximum value
    """
    k = _clean_val(key)
    self.measurements[k] = Value(value, min, max)

  def set_note(self, note):
    """
    @param note str : Note to add to test log
    """
    self.note = note


class TestData(object):
  """
  Stores test data to log into WG Test database
  """
  def __init__(self, name, desc, timestamp, reference, result):
    """
    @param name str : ID of test
    @param desc str : Description of test
    @param timestamp int : Timestamp test started
    @param reference str : Reference of test
    @param result str : Result of test. Ex: "PASS" or "FAIL"
    """
    self.name = name
    self.desc = desc
    self.reference = reference
    self.timestamp = timestamp
    self.result = result

    self.attachment = None

    self.note = ''

    self.subtests = []

  def set_attachment(self, content_type, fn):
    """
    @todo Attachment gets loaded into HDF. This should probably change
    @param content_type str : MIME type of attachment
    @param fn str : Filename
    """
    self.attachment = {"content_type":content_type, "filename":fn }

  def set_note(self, note):
    """
    @param note str : Note to add to test log
    """
    self.note = note

  def add_subtest(self, subtest):
    """
    Subtests are added to each test.
    @param subtest SubtestData : Subtest data
    """
    self.subtests.append(subtest)

def submit_log(client, testdata, attach_data):
  """
  Submits testing data to Invent system.
  
  @param client Invent : Invent client to load data
  @param testdata TestData : Test data to load
  @param attach_data str : File data (read from file)
  @return bool : True if loaded successfully
  """
  if not client.login():
    return False
  
  url = client.site + 'wgtest/api.py?Action.Add_TestData=1'

  hdf = obj2hdf("test", testdata)
  test_data = hdf.writeString() 

  fields = []
  fields.append(('Action.Add_TestData', '1'))
  
  files = []
  files.append(('testdata', 'testdata.hdf', test_data))

  if testdata.attachment:
    files.append(('attach', testdata.attachment['filename'], attach_data))
  
  log_input = attachment_help.build_request(client.site + "wgtest/api.py", fields, files)

  body = client.opener.open(log_input).read()
  
  # DEBUG
  #print body

  try:    
    ohdf = neo_util.HDF()
    ohdf.readString(body)
    
    logid = ohdf.getValue("logid", "")
    status = ohdf.getValue("status", "")
  except Exception, e:
    print 'Invalid HDF from server\n', body
    import traceback
    traceback.print_exc()
    return False
    
  if not status == '1':
    msg = ohdf.getValue("msg", "")
    print >> sys.stderr, "Unable to submit test data. Message: %s" % msg
    print >> sys.stderr, "HDF input:", test_data

  return status == '1'

