#! /usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

##\author Scott Hassan, Kevin Watts
##\brief Client for WG inventory system


import os, sys, string, time, getopt, re

import urllib2, cookielib

import mimetypes
import mimetools

import neo_cgi, neo_util
import simple_hdfhelp as hdfhelp

##\brief Checks if given serial number is a valid WG SN
def _is_serial_valid(reference):
  if len(reference) != 12:
    return False

  if not reference.startswith('680'):
    return False

  return True


## \brief Stores username and password, provides access to invent
##
## Performs all action relating to inventory system
## Will login automatically before all functions if needed
class Invent(object):
  ##@param username str : Username for WG invent system
  ##@param password str : Password for WG invent system
  def __init__(self, username, password, debug=False):
    self.username = username
    self.password = password

    self.debug = debug

    self.cj = cookielib.CookieJar()
    self.opener = urllib2.build_opener(urllib2.HTTPCookieProcessor(self.cj))

    self.loggedin = False
    self._logged_time = 0

    self.site = "http://invent.willowgarage.com/invent/"
    if debug:
      self.site = "http://cmi.willowgarage.com/invent/"

  ##\brief Logs into Invent. Returns true if successful
  def login(self):
    dt = time.time() - self._logged_time
    if self.loggedin==False or dt > 3600:
      return self._login()
    return True

  ## Login to inventory system with given username and password
  ## Returns True if successful, False if failure
  def _login(self):
    username = self.username
    password = self.password
    url = self.site + "login/signin0.py?Action.Login=1&username=%(username)s&password=%(password)s" % locals()

    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    self.loggedin = False
    self._logged_time = 0
    if body.find("Invalid Login") != -1:
      return False

    self.loggedin = True
    self._logged_time = time.time()
    return True

  ##\brief Reload BoM from PR2 SCDS. Loads barcode association tables
  ##
  ##\return bool : True if serial is valid, False if not
  def reload_pr2scds_bom(self):
    self.login()

    url = self.site + "invent/api.py?Action.reloadBom=1&" 
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()
    
    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return False

    val = hdf.getValue("CGI.out", "")
    return val.lower() == "true"

  ##\brief Check invent DB for serial, make sure it is valid
  ##
  ##\return bool : True if serial is valid, False if not
  def check_serial_valid(self, serial):
    if not _is_serial_valid(serial):
      return False

    self.login()

    url = self.site + "invent/api.py?Action.isItemValid=1&reference=%s" % (serial,)
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()
    
    i = string.find(body, "\n<!--")
    value = string.strip(body[:i])
    
    return value.lower() == "true"

  ##\brief Verifies that component is assembled and parts have passed Qual. 
  ##
  ##
  ## Checks that part is assembled against BoM. All sub-parts must be properly associated
  ## to the parent part. All sub-parts must have passed qualification.
  ##\param serial str : Serial number to check
  ##\param recursive bool : Check all sub-assemblies for assembly 
  ##\return True if part is assembled
  def check_assembled(self, serial, recursive = False):
    self.login()

    url = self.site + "invent/api.py?Action.checkAssembled=1&reference=%s" % (serial,)
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return False

    val = hdf.getValue("CGI.out", "")

    if not recursive:
      return val.lower() == "true"

    # If top-level isn't assembled, abort
    if not val.lower() == "true":
      return False

    subs = self.get_sub_items(serial)

    for sub in subs:
      if not self.check_assembled(sub, recursive):
        return False

    return True


  ##\brief Lookup item by a reference. 
  ##
  ## Item references are stored as key-values. Ex: { "wan0", "005a86000000" }
  ## This returns all items associated with a given reference value.
  ##\param str : Reference to lookup
  ##\return [ str ] : All items serial numbers that matched reference
  def lookup_by_reference(self, ref):
    self.login()

    url = self.site + "invent/api.py?Action.lookupByReference=1&reference=%s" % (ref,)
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return []

    rv = []
    for k,o in hdfhelp.hdf_ko_iterator(hdf.getObj("CGI.cur.items")):
      rv.append(o.getValue("reference", ""))

    return rv

  ## Return any references to an item. References are grouped by
  ## name, and are stored as NAME:REFERENCE,... under each item.
  ##@param key str : Serial number of item
  ##@return { str : str } : { name, reference }
  def getItemReferences(self, key):
    self.login()

    key = key.strip()

    url = self.site + "invent/api.py?Action.getItemReferences=1&key=%s" % (key,)
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return {}

    ret = {}
    for k,o in hdfhelp.hdf_ko_iterator(hdf.getObj("CGI.cur.refs")):
      ret[o.getValue("name", "")] = o.getValue("reference", "")
    
    return ret

  ##@brief Remove reference from item. 
  ##
  ##@param key str : Serial number of item
  ##@param name str : Reference name
  ##@return bool : True if reference was removed. Returns False if no reference found
  def remove_item_reference(self, key, name):
    self.login()

    key = key.strip()

    url = self.site + "invent/api.py?Action.removeItemReference=1&key=%s&name=%s" % (key,urllib2.quote(name))
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return False
      
    val = hdf.getValue("CGI.out", "")
    return val.lower() == "true"

    return True

  ##@brief Add reference to an item
  ##
  ##@param key str : Serial number of item
  ##@param name str : Reference name
  ##@param reference str : Reference value
  ##@return bool : True if reference is valid 
  def addItemReference(self, key, name, reference):
    self.login()

    key = key.strip()

    url = self.site + "invent/api.py?Action.addItemReference=1&key=%s&name=%s&reference=%s" % (key,urllib2.quote(name),urllib2.quote(reference))
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return False
    
    val = hdf.getValue("CGI.out", "")
    return val.lower() == "true"

    return True


  ## Generates Willow Garage mac address for item. Used for forearm cameras
  ## Does not return mac address
  ##@param key str : Serial number of item
  ##@param name str : Interface name (ex: "lan0")
  def generateWGMacaddr(self, key, name):
    self.login()

    key = key.strip()

    url = self.site + "invent/api.py?Action.generateWGMacaddr=1&key=%s&name=%s" % (key,urllib2.quote(name))
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

  ##\brief Sets notes of component
  ## Sets a note value for the component. Allows users to set the text of 
  ## a particular note if the noteid parameter is specified. Returns
  ## noteid to allow note edits. Returns None if error.
  ##@param reference str : Serial number of item
  ##@param note str : Text of item
  ##@param noteid int (optional) : Note ID, allows programmatic access to note text
  def setNote(self, reference, note, noteid = None):
    self.login()

    url = self.site + "invent/api.py?Action.AddNoteToItem=1&reference=%s&note=%s" % (reference, urllib2.quote(note))
    if noteid:
      url = url + "&noteid=%s" % noteid

    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    pat = re.compile("rowid=([0-9]+)")
    m = pat.search(body)
    if m:
      noteid = int(m.group(1))
      return noteid
    return None

  ##\brief Gets notes for an item.
  ##
  ##\param reference str : Serial number to check
  ##\param deleted bool : Retrieve deleted notes in addition to non-deleted notes
  ##\return { str : str } : Note ID to note text
  def get_item_notes(self, reference, deleted = False):
    self.login()

    url = self.site + "invent/api.py?Action.GetItemNotes=1&reference=%s" % (reference)
    if deleted:
      url += "&deleted=1"
    
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return {}

    ret = {}
    for k,o in hdfhelp.hdf_ko_iterator(hdf.getObj("CGI.cur.notes")):
      ret[o.getValue("noteid", "")] = o.getValue("note", "")

    return ret

  ##\brief Delete note for item. 
  ##
  ## Soft-delete only. Will return True if note has already been deleted
  ##\param noteid str : Note to delete
  ##\return bool : True if successful
  def delete_note(self, noteid):
    self.login()
    
    url = self.site + "invent/api.py?Action.DeleteNote=1&noteid=%s" % (noteid)

    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return False
    
    val = hdf.getValue("CGI.out", "")
    return val.lower() == "true"

  ##\brief Restore (un-delete) note for item. 
  ##
  ## Will return True if note has already been restored
  ##\param noteid str : Note to delete
  ##\return bool : True if successful
  def restore_note(self, noteid):
    self.login()
    
    url = self.site + "invent/api.py?Action.restoreNote=1&noteid=%s" % (noteid)

    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return False
    
    val = hdf.getValue("CGI.out", "")
    return val.lower() == "true"

  ##\brief Get information about note. 
  ##
  ## Returns tuple of note information as strings. Reference is serial number. 
  ## Date is UTC format. "deleted" is "1" for deleted, "0" if not.
  ##\param noteid str : Note ID
  ##\return (reference, note, username, date, deleted) (str, str, str, str, str) : 
  def get_note(self, noteid):
    self.login()

    url = self.site + "invent/api.py?Action.GetNote=1&noteid=%s" % (noteid)

    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return ('', '', '', '', '')
    
    ref = hdf.getValue("CGI.cur.item.reference", "")
    note = hdf.getValue("CGI.cur.note.note", "")
    user = hdf.getValue("CGI.cur.note.whom", "")
    date = hdf.getValue("CGI.cur.note.date", "")
    deleted = hdf.getValue("CGI.cur.note.deleted", "")

    return (ref, note, user, date, deleted)

  ##\brief Set value of component's key
  ##
  ## Set key-value of component. Ex: setKV(my_ref, 'Test Status', 'PASS')
  ##@param reference str : Serial number of component
  ##@param key str : Key (name)
  ##@param value str : Value
  ##@return True if set Key properly
  def setKV(self, reference, key, value):
    self.login()

    key = key.strip()
    value = value.strip()

    if not key:
      print >> sys.stderr, "Unable to set empty key into inventory system. Reference: %s, Value: %s" % (reference, value)
      return False

    if not value:
      print >> sys.stderr, "Unable to set empty value into inventory system. Reference: %s, Key: %s" % (reference, key)
      return False

    url = self.site + "invent/api.py?Action.setKeyValue=1&reference=%s&key=%s&value=%s" % (reference, urllib2.quote(key), urllib2.quote(value))

    fp = self.opener.open(url)
    fp.read()
    fp.close()

    return True

  ##\brief Delete key value for component.
  ##
  ## Delete key-value of component. 
  ##@param reference str : Serial number of component
  ##@param key str : Key (name)
  ##@return True if deleted key properly
  def deleteKV(self, reference, key):
    self.login()

    key = key.strip()

    if not key:
      print >> sys.stderr, "Unable to set empty key into inventory system. Reference: %s" % (reference)
      return False

    url = self.site + "invent/api.py?Action.deleteKeyValue=1&reference=%s&key=%s" % (reference, urllib2.quote(key))

    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return False

    val = hdf.getValue("CGI.cur.out", "")

    return val.lower() == "true"

  ##\brief Returns True if part has 'Test Status'='PASS', False otherwise
  def get_test_status(self, reference):
    return self.getKV(reference, 'Test Status') == 'PASS'

  ##\brief Return value of component's key
  ## Get key-value of component. Ex: 'PASS' = getKV(my_ref, 'Test Status')
  ##@param reference str : Serial number of component
  ##@param key str : Key (name)
  ##\return str : Value, or '' if key doesn't exist
  def getKV(self, reference, key):
    self.login()

    key = key.strip()

    if not key:
      raise ValueError, "the key is blank"

    url = self.site + "invent/api.py?Action.getKeyValue=1&reference=%s&key=%s" % (reference, urllib2.quote(key))

    fp = self.opener.open(url)
    value = fp.read()
    fp.close()

    i = string.find(value, "\n<!--")
    value = string.strip(value[:i])
    
    return value

  ##\brief List Key-Value pairs of component. 
  ##
  ##\param reference str : Serial number of component
  ##\return { str : str } : Key values of component
  def listKVs(self, reference):
    self.login()

    url = self.site + "invent/api.py?Action.listKeyValues=1&reference=%s" % (reference)

    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return {}

    ret = {}
    for k,o in hdfhelp.hdf_ko_iterator(hdf.getObj("CGI.cur.kvs")):
      ret[o.getValue("key", "")] = o.getValue("value", "")

    return ret

  ##\brief Adds attachment to component
  ##
  ## Adds file as attachment to component. Attachment it encoded to unicode,
  ## then uploaded. Mimetype is optional, but helps users view
  ## attachments in window. 
  ##@param reference str : Serial number of component
  ##@param name str : Attachment filename
  ##@param mimetype MIMEType : MIMEType of file
  ##@param attachment any : Attachement data
  ##@param note str : Note to add with attachment (description)
  ##@param aid str : Attachment ID. If set, attempts to overwrite attachment at ID
  ##\return str : Attachment ID of set attachment 
  def add_attachment(self, reference, name, mimetype, attachment, note = None, aid=None):
    self.login()

    if not name:
      raise ValueError, "the name is blank"

    theURL = self.site + "invent/api.py"

    fields = []
    fields.append(('Action.addAttachment', "1"))
    fields.append(('reference', reference))
    fields.append(('mimetype', mimetype))
    fields.append(('name', name))
    if note is not None:
      fields.append(('note', note))
    if aid is not None:
      fields.append(('aid', aid))

    files = []
    files.append(("attach", name, attachment))

    input = build_request(theURL, fields, files)

    response = self.opener.open(input).read()

    pat = re.compile("rowid=([0-9]+)")
    m = pat.search(response)
    if m is not None:
      aid = int(m.group(1))
      return aid
    return None

  ##\brief List all attachments for an item
  ##
  ##\return { str : str } : Attachment ID to filename
  def list_attachments(self, key):
    self.login()

    key = key.strip()

    url = self.site + "invent/api.py?Action.getAttachments=1&key=%s" % (key,)
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return {}

    ret = {}
    for k,o in hdfhelp.hdf_ko_iterator(hdf.getObj("CGI.cur.attachments")):
      ret[o.getValue("aid", "")] = o.getValue("name", "")
    
    return ret

  ##\brief Get attachment info. 
  ##
  ##\param aid str : Attachment ID
  ##\return ( str, str, str, str ) : reference, docname, username, note, date
  def get_attachment_info(self, aid):
    self.login()

    aid = aid.strip()

    url = self.site + "invent/api.py?Action.getAttachmentInfo=1&aid=%s" % (aid,)
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return ('', '', '', '')

    ref = hdf.getValue("CGI.cur.reference", "")
    docname = hdf.getValue("CGI.cur.attachment.name", "")
    user =  hdf.getValue("CGI.cur.attachment.whom", "")
    note = hdf.getValue("CGI.cur.attachment.note", "")
    date = hdf.getValue("CGI.cur.attachment.date", "")

    return (ref, docname, user, note, date)

  ##\brief Deletes attachment. Hard delete. 
  ##
  ## Returns true if attachment is deleted, or not found.
  ##\param aid str : Attachment ID
  ##\return bool : True if deleted
  def delete_attachment(self, aid):
    self.login()
    
    url = self.site + "invent/api.py?Action.deleteAttachment=1&aid=%s" % (aid)

    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return False
    
    val = hdf.getValue("CGI.out", "")
    return val.lower() == "true"

  ##\brief Returns list of sub items (references) for a particular parent
  ##
  ## In debug mode, Invent can return list of sub parts recursively
  ##\param reference str : WG PN of component or assembly
  ##\param recursive bool [optional] : Sub-sub-...-sub-parts of reference
  ##\return [ str ] : Serial number of sub-assemblies of component
  def get_sub_items(self, reference, recursive = False):
    self.login()
    
    reference = reference.strip()

    url = self.site + "invent/api.py?Action.getSubparts=1&reference=%s" % (reference)
    if recursive:
      url += "&recursive=1"

    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return {}

    ret = []
    for k,o in hdfhelp.hdf_ko_iterator(hdf.getObj("CGI.cur.items")):
      ret.append(o.getValue("reference", ""))
    
    return ret

  ##\brief Returns parent item (serial number) or location. 
  ##
  ## Parent may be a location, such as "Recieving", instead of a serial number.
  ##\param reference str : Serial number to check
  ##\return str : Serial number of parent. None if no parent found
  def get_parent_item(self, reference):
    self.login()

    url = self.site + "invent/api.py?Action.getParentItem=1&reference=%s" % (reference)
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    try:
      hdf.readString(body)
    except Exception, e:
      print >> sys.stderr, 'Unable to parse HDF output from inventory system. Output:\n%s' % body
      return None

    parent = hdf.getValue("CGI.cur.parent", "")
    if not parent:
      return None

    return parent



## -------------------------------------------------------------

def build_request(theurl, fields, files, txheaders=None):
  content_type, body = encode_multipart_formdata(fields, files)
  if not txheaders: txheaders = {}
  txheaders['Content-type'] = content_type
  txheaders['Content-length'] = str(len(body))
  return urllib2.Request(theurl, body, txheaders)

def encode_multipart_formdata(fields, files, BOUNDARY = '-----'+mimetools.choose_boundary()+'-----'):
    """ 
    Encodes fields and files for uploading.

    fields is a sequence of (name, value) elements for regular form fields - or a dictionary.

    files is a sequence of (name, filename, value) elements for data to be uploaded as files.

    Return (content_type, body) ready for urllib2.Request instance

    You can optionally pass in a boundary string to use or we'll let mimetools provide one.

    """    

    CRLF = '\r\n'

    L = []

    if isinstance(fields, dict):
        fields = fields.items()

    for (key, value) in fields:
        L.append('--' + BOUNDARY)
        L.append('Content-Disposition: form-data; name="%s"' % key)
        L.append('')
        L.append(value)

    for (key, filename, value) in files:
        #encoded = value.encode('base64')
        encoded = value
        filetype = mimetypes.guess_type(filename)[0] or 'application/octet-stream'
        L.append('--' + BOUNDARY)
        L.append('Content-Disposition: form-data; name="%s"; filename="%s"' % (key, filename))
        L.append('Content-Length: %s' % len(encoded))
        L.append('Content-Type: %s' % filetype)
        L.append('Content-Transfer-Encoding: binary')
        L.append('')
        L.append(encoded)

    L.append('--' + BOUNDARY + '--')
    L.append('')
    body = CRLF.join([str(l) for l in L])

    content_type = 'multipart/form-data; boundary=%s' % BOUNDARY        # XXX what if no files are encoded

    return content_type, body


