#!/usr/bin/env python
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


import string, os
import neo_util

def loopHDF(hdf, name=None):
  results = []
  if name: o = hdf.getObj(name)
  else: o = hdf
  if o:
    o = o.child()
    while o:
      results.append(o)
      o = o.next()
  return results


def loopKVHDF(hdf, name=None):
  results = []
  if name: o = hdf.getObj(name)
  else: o = hdf
  if o:
    o = o.child()
    while o:
      results.append((o.name(), o.value()))
      o = o.next()
  return results


class hdf_iterator:
  def __init__(self, hdf):
    self.hdf = hdf
    self.node = None
    if self.hdf:
      self.node = self.hdf.child()

  def __iter__(self): return self

  def next(self):
    if not self.node:
      raise StopIteration

    ret = self.node
    self.node = self.node.next()
      
    return ret

class hdf_kv_iterator(hdf_iterator):
  def next(self):
    if not self.node: raise StopIteration

    ret = (self.node.name(), self.node.value())
    self.node = self.node.next()
      
    return ret

class hdf_key_iterator(hdf_iterator):
  def next(self):
    if not self.node: raise StopIteration

    ret = self.node.name()
    self.node = self.node.next()
      
    return ret

class hdf_ko_iterator(hdf_iterator):
  def next(self):
    if not self.node: raise StopIteration

    ret = (self.node.name(), self.node)
    self.node = self.node.next()
      
    return ret
  

def hdfExportDict(prefix, hdf, dict):
  for k,v in dict.items():
    hdf.setValue(prefix + "." + str(k), str(v))


def hdfExportList(prefix, hdf, list):
  n = 0
  for item in list:
    n = n + 1
    hdf.setValue(prefix + "." + str(n), str(item))







