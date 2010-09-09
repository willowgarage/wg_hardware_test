#!/usr/bin/env python

import roslib; roslib.load_manifest('wg_invent_client')

import sys, getpass

from wg_invent_client import Invent

def print_usage():
    print 'Loads mac address into Invent for a given robot'
    print
    print 'load_mac_addr.py 10XX USERNAME ADDRESS'
    print 'Asks for password'
    
    return

if __name__ == '__main__':
    if len(sys.argv) != 4:
        print_usage()
        sys.exit(1)

    print >> sys.stderr, "Make sure you have the right robot. Enter Invent password"

    sn = '68029670' + sys.argv[1]
    user = sys.argv[2]
    addr = sys.argv[3].replace(':', '')

    if not len(addr) == 12:
        print >> sys.stderr, "Mac address %s is not valid. Please retry." % addr
        sys.exit(1)

    passwd = getpass.getpass()

    iv = Invent(user, passwd)
    
    if not iv.login():
        print >> sys.stderr, "Unable to login to Invent. Try again."
        sys.exit(1)

    if not iv.check_serial_valid(sn):
        print >> sys.stderr, "Serial %s is invalid. Robot must be entered in as '10XX'" % sn
        sys.exit(1)

    iv.addItemReference(sn, 'wan0', addr)

    print 'Loaded wan0 mac address for %s' % sn
