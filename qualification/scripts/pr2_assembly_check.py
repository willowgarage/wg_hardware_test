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
##\brief Checks that all devices in a PR2 are properly associated to robot

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

import os, sys
from optparse import OptionParser

import getpass
import subprocess

from wg_invent_client import Invent

class GetIDException(Exception): pass

# Document part numbers do not have to be associated to PR2
DOC_PNS = ['6803642', '6803643', '6803708', '6805005' ]

def _ignore_pn(pn):
    for doc in DOC_PNS:
        if pn.find(doc) > -1:
            return True
    return False

if __name__ == '__main__':
    parser = OptionParser(usage="%prog -u USERNAME -r ROBOT")
    parser.add_option('-u', '--username', action="store", dest="username",
                      default=None, metavar="USERNAME",
                      help="Username for WG inventory system")
    parser.add_option('-r', '--robot', action="store", dest="robot",
                      default=None, metavar="ROBOT",
                      help="Robot SN (10XX) to check")

    options,args = parser.parse_args()
    if not options.username:
        parser.error("Must provide username to WG inverntory system")
    if not options.robot:
        parser.error("Must provide valid robot SN to log")

    robot = '68029670' + options.robot
    
    if not len(robot) == 12:
        parser.error("%s is not a valid robot serial number" % options.robot)

    print 'Enter your password to the Willow Garage Inventory system'
    my_pass = getpass.getpass()

    iv = Invent(options.username, my_pass)
    if not iv.login():
        parser.error("Must provide valid username and password to WG inventory system")
    if not iv.check_serial_valid(robot):
        parser.error("Robot serial number %s is invalid" % options.robot)

    print 'Getting parts from Invent'
    my_parts = iv.get_sub_items(robot, True)
    print 'Making sure components assembled properly'
    print ''
    print ''

    ok = True
    for pt in my_parts:
        if _ignore_pn(pt):
            continue

        if not iv.check_assembled(pt):
            print >> sys.stderr, "Part %s is not assembled properly." % pt
            ok = False

    if ok:
        print "Robot is properly associated in Invent"
        sys.exit()
    
    print >> sys.stderr, "Robot is not properly assembled."
    sys.exit(1)
