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
##\brief Checks that all devices in a PR2 are properly associated to robot

PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

import os, sys
from optparse import OptionParser

import getpass
import subprocess

from wg_invent_client import Invent

class GetIDException(Exception): pass

WGE100_PN = '68050' # Prefix for all wge100 camera PN's

FWPROG_PATH = os.path.join(roslib.packages.get_pkg_dir(PKG), 'fwprog')
PR2_GRANT = 'pr2_grant'

def get_wge100_serials():
    cmd = '%s/bin/discover lan1' % roslib.packages.get_pkg_dir('wge100_camera')
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr = subprocess.PIPE, shell=True)
    (o, e) = p.communicate()
    if p.returncode != 0:
        raise GetIDException("Unable to run \"discover\" to find camera serials!")
    
    serials = []

    lines = o.split('\n')
    for ln in lines:
        if len(ln.split()) < 5:
            continue

        serial_start = 'serial://'
        idx = ln.find(serial_start)
        if idx < 0:
            continue
        ser = ln[idx + len(serial_start): idx + len(serial_start) + 7]
        
        serial = WGE100_PN + ser

        serials.append(serial)    

    return serials

def get_prosilica_ref():
    cmd = '%s/bin/ListAttributes' % roslib.packages.get_pkg_dir('prosilica_gige_sdk')
    p = subprocess.Popen([cmd, '10.68.0.20'], stdout = subprocess.PIPE, 
                         stderr= subprocess.PIPE)
    
    (o, e) = p.communicate()
    if p.returncode != 0:
        raise GetIDException("Unable to get Prosilica ID")

    lines = o.split('\n')
    for ln in lines:
        if ln.find('UniqueId') == -1:
            continue
        
        return ln.split()[-1]
    
    return ''

def get_imu_ref():
    cmd = '%s/get_id' % roslib.packages.get_pkg_dir('microstrain_3dmgx2_imu')
    p = subprocess.Popen([cmd, '/etc/ros/sensors/imu', '-q'], stdout = subprocess.PIPE, 
                         stderr= subprocess.PIPE)
    
    (o, e) = p.communicate()
    if p.returncode != 0:
        raise GetIDException("Unable to get IMU device ID")

    lines = o.split('\n')
    return lines[0].strip()


def get_hk_refs():
    ids = []
    for port in [ 'base_hokuyo', 'tilt_hokuyo' ]:
        cmd = 'source /etc/ros/setup.sh; rosrun hokuyo_node getID /etc/ros/sensors/' + port
        p = subprocess.Popen(['ssh', 'c2', cmd], stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE)
        (o, e) = p.communicate()
        if p.returncode != 0:
            raise GetIDException("Unable to get HK ID for %s" % port)
        
        if len(o.split()) < 3:
            raise GetIDException("Unable to get HK ID for %s. Output invalid: %s" % (port, o))

        hk_id = o.split()[-1]
        ids.append(hk_id)

    return ids 

def get_mcb_serials():
    serials = []

    count_cmd = PR2_GRANT + ' ' + os.path.join(FWPROG_PATH, 'eccount') + ' -iecat0'

    count = subprocess.call(count_cmd, shell=True)
    if count != 37:
        raise Exception("Invalid MCB count. Should be 37. Count: %d" % count)

    ##\todo Use motorconf to pull MCB SN's so it's not so slow
    for dev in range(1, 38):
        cmd = PR2_GRANT + ' ' + os.path.join(FWPROG_PATH, 'device') + ' -iecat0 -K -p %d' % dev
        p = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE,
                             shell = True)
        o, e = p.communicate()
        if p.returncode != 0:
            raise Exception("Unable to recover serial number from device %d. Returned %d" % (dev, p.returncode))
        
        for ln in o.split('\n'):
            if ln.startswith('serial :'):
                sn = ln[9:].strip()
                if sn not in serials:
                    serials.append(sn)

    return serials

def get_wan0_mac():
    p = subprocess.Popen(['ifconfig', 'wan0'], stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE)

    (o, e) = p.communicate()
    if p.returncode != 0:
        raise GetIDException("Unable to recover wan0 mac address")

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
    parser = OptionParser(usage="%prog -u USERNAME -r ROBOT\nMust be run on c1 of PR2")
    parser.add_option('-u', '--username', action="store", dest="username",
                      default=None, metavar="USERNAME",
                      help="Username for WG inventory system")
    parser.add_option('-r', '--robot', action="store", dest="robot",
                      default=None, metavar="ROBOT",
                      help="Robot SN (10XX) to store data.")

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

    print 'Pulling devices from robot'

    ok = True

    wan0_mac = get_wan0_mac()
    imu_id = get_imu_ref()
    prosilica = get_prosilica_ref()
    hks = get_hk_refs()
    wge100s = get_wge100_serials()
    mcbs = get_mcb_serials()

    print 'Getting parts from Invent'
    my_parts = iv.get_sub_items(robot, True)

    # Check wan0 mac address
    mac_serials = iv.lookup_by_reference(wan0_mac)
    if not len(mac_serials) == 1 and mac_serials[0] == robot:
        print >> sys.stderr, "Mac address for wan0 doesn't match robot. Mac address: %s" % wan0_mac
        ok = False

    # Check IMU
    serials = iv.lookup_by_reference(imu_id)
    if not len(serials) == 1:
        print >> sys.stderr, "Invalid serial numbers listed for IMU. %s" % ', '.join(serials)
        ok = False

    if len(serials) == 1 and serials[0] not in my_parts:
        print >> sys.stderr, "IMU serial %s is not listed in Invent. ID: %s" % (serials[0], imu_id)
        ok = False

    # Check prosilica
    serials = iv.lookup_by_reference(prosilica)
    if not len(serials) == 1:
        print >> sys.stderr, "Invalid serial numbers listed for Prosilica camera. %s" % ', '.join(serials)
        ok = False

    if len(serials) == 1 and serials[0] not in my_parts:
        print >> sys.stderr, "Prosilica serial %s is not listed in Invent. ID: %s" % (serials[0], prosilica)
        ok = False

    # Check wge100 cameras
    for wge in wge100s:
        if not wge in my_parts:
            print >> sys.stderr, "Camera %s was not found in parts list" % wge
            ok = False

    if not len(wge100s) == 6:
        print >> sys.stderr, "Not all wge100's found on robot. Expected 6, found %d" % len(wge100s)
        ok = False
    
    # Check HK's
    for hk in hks:
        serials = iv.lookup_by_reference(hk)
        if not len(serials) == 1:
            print >> sys.stderr, "Invalid serial numbers for HK reference %s. Serials: %s" % (hk, ', '.join(serials))
            ok = False
            continue

        if not serials[0] in my_parts:
            print >> sys.stderr, "Hokuyo %s not found in parts list. Device ID: %s" % (serials[0], hk)
            ok = False

    # Check etherCAT chain
    for mcb in mcbs:
        if not mcb in my_parts:
            print >> sys.stderr, "EtherCAT device %s was not found in parts list" % mcb
            ok = False

    if ok:
        print "All devices matched robot."
        sys.exit()
    
    print >> sys.stderr, "Error matching devices to robot"
    sys.exit(1)
