#! /usr/bin/env python

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

## Accessor functions for retrieving data
def get_test_logs(client, key):
  """
  Returns all test logs for a particular serial number

  @param client Invent : Invent client to load data
  @param key str : Reference to check
  @return { str : ( str, str) } : { Log ID : (Test ID, timestamp) }
  """
  if not client.login():
    print >> sys.stderr, 'Unable to login to Invent'
    return {}

  key = key.strip()

  url = client.site + 'wgtest/api.py?Action.Get_TestLogs=1&key=%s' % (key,)
  
  body = client.opener.open(url).read()

  #print body
  
  hdf = neo_util.HDF()
  hdf.readString(body)
  
  rv = {}
  for k, o in hdfhelp.hdf_ko_iterator(hdf.getObj("CGI.cur.test")):
    rv[o.getValue("logid", "")] = (o.getValue("testid", ""), o.getValue("timestamp", "") )

  return rv




def get_subtest_info(client, subtestid):
  """
  Retrieve name, description for particular subtest ID
  @param subtestid str : ID of subtest
  @return (str, str): Name of subtest, testid
  """
  if not client.login():
    print >> sys.stderr, 'Unable to login to Invent'
    return ('', '')

  subtestid = str(subtestid).strip()

  url = client.site + 'wgtest/api.py?Action.Get_SubtestInfo=1&subtestid=%s' % (subtestid,)
  
  body = client.opener.open(url).read()

  #print body
  
  hdf = neo_util.HDF()
  hdf.readString(body)
  
  name = hdf.getValue("subtest.name", "")
  testid = hdf.getValue("subtest.testid", "")

  return name, testid



def get_test_info(client, testid):
  """
  Retrieve name, description for particular test ID
  @param testid str : ID of test
  @return (str, str): Name, description of test
  """
  if not client.login():
    print >> sys.stderr, 'Unable to login to Invent'
    return ('', '')

  testid = testid.strip()

  url = client.site + 'wgtest/api.py?Action.Get_TestInfo=1&testid=%s' % (testid,)
  
  body = client.opener.open(url).read()

  #print body
  
  hdf = neo_util.HDF()
  hdf.readString(body)
  
  name = hdf.getValue("test.name", "")
  desc = hdf.getValue("test.description", "")

  return name, desc

def get_testid_by_name(client, name):
  """
  @param name str : Short name of test. Ex: 'caster-post'
  @return str: Test ID
  """
  if not client.login():
    print >> sys.stderr, 'Unable to login to Invent'
    return ''

  name = name.strip()

  url = client.site + 'wgtest/api.py?Action.Get_TestIdByName=1&testname=%s' % (name,)
  
  body = client.opener.open(url).read()

  #print body
  
  hdf = neo_util.HDF()
  hdf.readString(body)
  
  return hdf.getValue("test.testid", "")


  

# debug only
def get_run_info(client, runid):
  """

  @param client Invent : Invent client to load data
  @return (str, str, str, str) : Log ID, Version ID, result, note
  """
  if not client.login():
    return ('', '', '', '')

  runid = str(runid).strip()

  url = client.site + 'wgtest/api.py?Action.Get_RunData=1&runid=%s' % (runid,)
  
  body = client.opener.open(url).read()

  #print body
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)

  logid = ohdf.getValue("run.logid", "")
  versionid = ohdf.getValue("run.versionid", "")
  result = ohdf.getValue("run.result", "")
  note = ohdf.getValue("run.note", "")

  return logid, versionid, result, note


def get_log_data(client, logid):
  """
  @param logid str : Log ID to check
  @return (str, str, str, str, str, str, str, int) : Test ID, serial number, result, note, docref, attachment, timestamp
  """
  if not client.login():
    return None

  logid = str(logid).strip()

  url = client.site + 'wgtest/api.py?Action.Get_LogData=1&logid=%s' % (logid,)
  
  body = client.opener.open(url).read()

  #print body
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)

  testid = ohdf.getValue("log.testid", "")
  reference = ohdf.getValue("log.reference", "")
  result = ohdf.getValue("log.result", "")
  whom = ohdf.getValue("log.whom", "")
  note = ohdf.getValue("log.note", "")
  docref = ohdf.getValue("log.docref", "")
  attachment = ohdf.getValue("log.attach_name", "")
  timestamp = int(ohdf.getValue("log.timestamp", "0"))

  return (testid, reference, result, whom, note, docref, attachment, timestamp)

def get_log_runs(client, logid):
  """
  @param logid str : Log ID to check
  @return [ str ] : Runid's for log
  """
  if not client.login():
    return []

  logid = str(logid).strip()

  url = client.site + 'wgtest/api.py?Action.Get_LogData=1&logid=%s' % (logid,)
  
  body = client.opener.open(url).read()

  #print body
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)

  rv = []
  for k, o in hdfhelp.hdf_ko_iterator(ohdf.getObj("runs")):
    rv.append(o.getValue("runid", ""))

  return rv


def get_version_info(client, versionid):
  """
  
  @param versionid str : Version ID to check
  @return (str, str, str) : Name, Subtest ID, Test ID
  """
  if not client.login():
    return ('', '', '')

  url = client.site + 'wgtest/api.py?Action.Get_VersionInfo=1&versionid=%s' % (str(versionid))
  
  body = client.opener.open(url).read()
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)

  return (ohdf.getValue("subtest.name", ""), ohdf.getValue("subtest.subtestid", ""), ohdf.getValue("subtest.testid", ""))
  

def get_version_params(client, versionid):
  """
  @param versionid str : Version ID of subtest
  @return { str: str } : Key-values of all parameters in subtest version
  """
  if not client.login():
    return {}

  url = client.site + 'wgtest/api.py?Action.Get_VersionParams=1&versionid=%s' % (str(versionid))
  
  body = client.opener.open(url).read()
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)

  rv = {}
  for k, o in hdfhelp.hdf_ko_iterator(ohdf.getObj("params")):
    if not o.getValue("key", ""):
      continue
    rv[o.getValue("key", "")] = o.getValue("value", "")
    
  return rv

def get_subtest_versions(client, subtestid):
  """
  @param subtestid str : Subtest ID to check
  @return [ str ] : Version ID's of all versions of this subtest
  """
  if not client.login():
    return []

  url = client.site + 'wgtest/api.py?Action.Get_SubtestVersions=1&subtestid=%s' % (str(subtestid))
  
  body = client.opener.open(url).read()
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)

  rv = []
  for k, o in hdfhelp.hdf_ko_iterator(ohdf.getObj("versions")):
    if not o.getValue("versionid", ""):
      continue
    rv.append(o.getValue("versionid", ""))

  return rv



def get_run_measurements(client, runid):
  """
  @runid str : Run ID to check
  @return { str: (str, str, str) } : Measurement value, min, max by key
  """
  if not client.login():
    return None

  url = client.site + 'wgtest/api.py?Action.Get_RunMeasurements=1&runid=%s' % (str(runid))
  
  body = client.opener.open(url).read()
  
  #print body
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)

  rv = {}
  for k, o in hdfhelp.hdf_ko_iterator(ohdf.getObj("measurements")):
    rv[o.getValue("key", "")] = (o.getValue("value", ""), o.getValue("min", ""), o.getValue("max", ""))

  return rv

# debug only
def get_tests_by_name(test_name, client):
  if not client.login():
    return None

  test_name = test_name.strip()

  url = client.site + 'wgtest/api.py?Action.Get_TestsByName=1&test_name=%s' % (urllib2.quote(test_name))
  
  body = client.opener.open(url).read()

  #print body
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)


  rv = {}
  for k, o in hdfhelp.hdf_ko_iterator(ohdf.getObj("values")):
    rv[o.getValue("id", "")] = o.getValue("params", "")

  return rv

def get_subtest_results(client, subtestid):
  """
  @param subtestid str : Subtest
  @return { str : ( str, str ) } : { Run ID : ( PASS/FAIL, Version ID ) }
  """
  if not client.login():
    return {}

  url = client.site + 'wgtest/api.py?Action.Get_SubtestResults=1&subtestid=%s' % (str(subtestid))
  
  body = client.opener.open(url).read()

  #print body
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)

  rv = {}
  for k, o in hdfhelp.hdf_ko_iterator(ohdf.getObj("results")):
    rv[o.getValue("runid", "")] = (o.getValue("result", ""), o.getValue("versionid", ""))

  return rv

def get_version_results(client, versionid):
  """
  @param subtestid str : Subtest
  @return { str : ( str, str ) } : { Run ID : ( PASS/FAIL, Version ID ) }
  """
  if not client.login():
    return []

  url = client.site + 'wgtest/api.py?Action.Get_VersionResults=1&versionid=%s' % (str(versionid))
  
  body = client.opener.open(url).read()

  #print body
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)

  rv = {}
  for k, o in hdfhelp.hdf_ko_iterator(ohdf.getObj("results")):
    rv[o.getValue("runid", "")] = (o.getValue("result", ""), o.getValue("versionid", ""))

  return rv


def get_measurement_history(client, key, subtestid, versionid = None):
  """
  Return history of all subtest measurements

  @param key str : Measurement name (ex: 'Negative velocity avg.')
  @param subtestid str : Subtest ID 
  @param versionid str : Version ID. Specify specific version only if desired
  @return { str : (str, str, str, str, str) } : { Run ID : (min, max, value, timestamp, versionid) }
  """
  if not client.login():
    return None

  
  url = client.site + 'wgtest/api.py?Action.Get_MeasurementHistory=1&key=%s&subtestid=%s' % (urllib2.quote(key), str(subtestid))
  if versionid is not None:
    url += "&versionid=%s" % str(versionid)

  body = client.opener.open(url).read()

  #print body
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)

  rv = {}
  for k, o in hdfhelp.hdf_ko_iterator(ohdf.getObj("measurement")):
    rv[o.getValue("runid", "")] = (o.getValue("min", ""), o.getValue("max", ""), o.getValue("value", ""), o.getValue("timestamp", ""), o.getValue("versionid", ""))

  return rv



def is_item_tested(client, key):
  """
  @param key str : Serial number to check
  @return bool : True if fully tested
  """
  if not client.login():
    return None

  url = client.site + 'wgtest/api.py?Action.IsItemTested=1&key=%s' % (urllib2.quote(key))

  body = client.opener.open(url).read()

  #print body                                                                                                          

  ohdf = neo_util.HDF()
  ohdf.readString(body)

  return ohdf.getValue("tested", "") == "true"

def get_test_log_sequence(client, key):
  """
  @key str : Serial number of component to check
  @return [ (str, str, int) ] : Log ID, Test ID, timestamp of log
  """
  if not client.login():
    return None

  url = client.site + 'wgtest/api.py?Action.GetTestLogSequence=1&key=%s' % (urllib2.quote(key))

  body = client.opener.open(url).read()

  #print body                                                                                                          

  ohdf = neo_util.HDF()
  ohdf.readString(body)

  rv = []
  for k, o in hdfhelp.hdf_ko_iterator(ohdf.getObj("sequence")):
    rv.append((o.getValue("logid", ""), o.getValue("testid", ""), int(o.getValue("timestamp", "0")) ) )

  return rv

def get_reqd_sequence(client, part):
  """
  
  @param part str : Part number (ex: '4108') of device to get sequence for
  @return [ str ] : Test ID's of test sequence
  """
  if not client.login():
    return None

  url = client.site + 'wgtest/api.py?Action.GetRequiredSequence=1&part=%s' % (urllib2.quote(part))

  body = client.opener.open(url).read()

  #print body                                                                                                          

  ohdf = neo_util.HDF()
  ohdf.readString(body)

  rv = []
  for k, o in hdfhelp.hdf_ko_iterator(ohdf.getObj("sequence")):
    rv.append(o.getValue("testid", ""))
    
  return rv
