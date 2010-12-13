#!/usr/bin/env python
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\author Kevin Watts
##\brief Logs Results of PR2 burn in test to CSV file, uploads to invent

from __future__ import with_statement

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)

from life_test.test_param import LifeTest
from life_test.test_record import TestRecord
from pr2_self_test_msgs.msg import TestStatus

import sys, os
import rospy

import threading
import getpass
from optparse import OptionParser
from wg_invent_client import Invent

class PR2TestLogger:
    def __init__(self, robot_serial, iv, output_file = None, submit = True, email = True):
        my_test = LifeTest(short='PR2', testid='pr2-burn-test', name='PR2 Burn in Test', 
                           desc='PR2 Burn in Test', serial='6802967',
                           duration=1, launch_script='pr2_test/pr2_burn_in_test.launch', 
                           test_type='Burn in', power=False)

        self._out_file = output_file
        self._record = TestRecord(my_test, robot_serial, csv_name = self._out_file, send_email = email)

        self._mutex = threading.Lock()

        self._iv = iv
        self._serial = robot_serial
        self._closed = False
        self._submit = submit

        self._record.update(False, False, False, 'Started Logging', '')

        self._status_sub = rospy.Subscriber('test_status', TestStatus, self._status_cb)

    def _status_cb(self, msg):
        with self._mutex:
            self._record.update(True, msg.test_ok == 0, False, '', msg.message)

    def close(self):
        if self._closed:
            return 

        # Don't log if we didn't start running
        if self._record.get_cum_time() < 1:
            print >> sys.stderr, "No robot burn in data record. Unable to send to inventory system"
            return

        self._record.update(False, False, False, 'Stopping robot', '')

        if not self._submit:
            print "Log recorded: %s. Not submitting to Invent" % self._record.log_file
            return

        if self._record.load_attachments(self._iv):
            print 'Submitted log to inventory system'
        else:
            print >> sys.stderr, "Unable to submit log. Load attachments to invent manually"

        self._closed = True
    

if __name__ == '__main__':
    parser = OptionParser(usage="%prog -u USERNAME -r ROBOT", prog="pr2_test_logger.py")
    parser.add_option('-u', '--username', action="store", dest="username",
                      default=None, metavar="USERNAME",
                      help="Username for WG inventory system")
    parser.add_option('-r', '--robot', action="store", dest="robot",
                      default=None, metavar="ROBOT",
                      help="Robot SN (10XX) to store data.")
    parser.add_option('-o', '--output', action="store", dest="output",
                      default=None, metavar="OUTPUT_FILE",
                      help="Log file to store data. Appended to existing file")
    parser.add_option('-n', '--no-submit', action="store_true", dest="no_submit",
                      default=False, help="Don't submit to Inventory system")
    parser.add_option('--no-email', action="store_true", dest="no_email",
                      default=False, help="Don't email updates")
    
    options,args = parser.parse_args()
    if not options.username and not options.no_submit:
        parser.error("Must provide username to WG inverntory system")
    if not options.robot:
        parser.error("Must provide valid robot SN to log")

    robot = '68029670' + options.robot
    
    if not options.no_submit:
        print 'Enter your password to the Willow Garage Inventory system'
        my_pass = getpass.getpass()

        iv = Invent(options.username, my_pass)
        if not iv.login():
            parser.error("Must provide valid username and password to WG inventory system")
        if not iv.check_serial_valid(robot):
            parser.error("Robot serial number %s is invalid" % options.robot)
    else:
        iv = None

    rospy.init_node('pr2_test_logger') # , disable_signals = True)

    pr2_logger = PR2TestLogger(robot, iv, options.output, not options.no_submit, not options.no_email)
    rospy.on_shutdown(pr2_logger.close)
    
    print "Logging PR2 burn in test status..."
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down logging, reporting to inventory system"
        pr2_logger.close()
    except Exception, e:
        import traceback
        traceback.print_exc()
        pr2_logger.close()
