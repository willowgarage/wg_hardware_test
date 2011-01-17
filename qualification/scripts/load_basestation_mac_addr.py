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

import roslib; roslib.load_manifest('qualification')

import sys, getpass

from wg_invent_client import Invent

from optparse import OptionParser
import subprocess

class GetIDException(Exception): pass

def print_usage():
    print 'Loads mac address into Invent for a given basestation'
    print
    print 'load_basestation_mac_addr.py 10XX USERNAME'
    print 'Asks for password'
    
    return

def get_iface_mac(iface):
    p = subprocess.Popen(['ifconfig', iface], stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE)

    (o, e) = p.communicate()
    if p.returncode != 0:
        raise GetIDException("Unable to recover %s mac address" % iface)

    lines = o.split('\n')
    first = lines[0]

    words = first.split()
    if len(words) < 5 or not words[3].strip() == 'HWaddr':
        raise GetIDException("Unable to parse mac address output: %s. Got: %s. Length: %d" % (first, words[3], len(words)))
    mac = words[4].replace(':', '').lower()

    if not len(mac) == 12:
        raise GetIDException("Unable to parse mac address output: %s. Recovered: %s" % (first, mac))
    
    return mac

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print_usage()
        sys.exit(1)

    sn = '68037600' + sys.argv[1]
    user = sys.argv[2]

    print "Enter Invent password"
    passwd = getpass.getpass()

    iv = Invent(user, passwd)
    
    if not iv.login():
        print >> sys.stderr, "Unable to login to Invent. Try again."
        sys.exit(1)

    if not iv.check_serial_valid(sn):
        print >> sys.stderr, "Serial %s is invalid. Robot must be entered in as '10XX'" % sn
        sys.exit(1)

    for iface in ('wan0', 'lan0'):
        addr =  get_iface_mac(iface)
        if not iv.addItemReference(sn, iface, addr):
            raise Exception("Unable to load address %s to iface %s" % (addr, iface))

    print 'Loaded mac address for %s' % sn
