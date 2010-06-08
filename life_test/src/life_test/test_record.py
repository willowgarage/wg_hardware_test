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
##\brief Records data from LifeTest into a CSV file

from __future__ import with_statement

PKG = 'life_test'
import roslib
roslib.load_manifest(PKG)

import csv
import os, sys, math

import rospy

from socket import gethostname

from writing_core import *

LOG_UPDATE = 7200 # Update log if no entries before this time

class TestState(object):
    """
    Tracks state of running/stopped tests
    """
    def __init__(self, launched, running, stale, monitor_msg, note = ''):
        self.launched = launched
        self.running = running
        self.stale = stale
        self.monitor_msg = monitor_msg
        self.note = note


def _get_csv_header_lst(params):
    hdr = [ 'Time', 'Status', 'Message', 'Elapsed', 'Cum. Time',
            'Num. Halts', 'Num. Events' ]

    for p in params:
        if p.is_rate():
            hdr.append('Cum. %s' % p.get_name())

    hdr.append('Note')

    return hdr


class LogEntry(object):
    """
    Entry in test log. Gets written to CSV output file.
    """
    def __init__(self, stamp, elapsed, cum_time, status, message, params, halts, events, note):
        self.stamp = stamp

        self.status = status
        self.message = message
        self.params = params
        self.note = note

        # Time in seconds
        self.elapsed = elapsed
        self.cum_time = cum_time

        self.halts = halts
        self.events = events

    def write_to_lst(self):
        lst = [ format_localtime(self.stamp), self.status, self.message, 
                get_duration_str(self.elapsed), 
                get_duration_str(self.cum_time), self.halts, self.events ]
 
        for p in self.params:
            if p.is_rate():
                lst.append(p.get_value() * self.cum_time)

        lst.append(self.note)

        return lst

        

##\brief Updates CSV record with state changes for a test
##
## 
class TestRecord:
    ##\param test LifeTest : Test type, params
    ##\param serial str : Serial number of DUT
    def __init__(self, test, serial, file_path = None):
        self._start_time = rospy.get_time()
        self._cum_seconds = 0
        self._last_update_time = rospy.get_time()
        self._was_running = False
        self._was_launched = False
        self._num_events = 0
        self._num_halts = 0

        self._serial = serial
        self._test_name = test.get_name()

        self._test = test

        self._log_entries = []

        self._last_log_time = self._last_update_time

        self._cum_data = {}
        for param in test.params:
            if param.is_rate():
                self._cum_data[param.get_name()] = 0

        csv_name = format_localtime(self._start_time) + '_' + \
            str(self._serial) + '_' + self._test_name + '.csv'
        csv_name = csv_name.replace(' ', '_').replace('/', '__')

        if not file_path:
            file_path = os.path.join(roslib.packages.get_pkg_dir(PKG), 'logs')
        self.log_file = os.path.join(file_path, csv_name)

        
        with open(self.log_file, 'ab') as f:
            log_csv = csv.writer(f)
            log_csv.writerow(_get_csv_header_lst(self._test.params))

    def get_elapsed(self):
        elapsed = rospy.get_time() - self._start_time
        return elapsed

    def get_cum_time(self):
        return self._cum_seconds

    def get_active_str(self):
        return get_duration_str(self._cum_seconds)

    def get_elapsed_str(self):
         return get_duration_str(self.get_elapsed())

    # TODO
    def get_cycles(self, name = None):
        if not self._cum_data.has_key(name):
            kys = self._cum_data.keys()
            kys.sort()
            return self._cum_data[kys[0]]
        return self._cum_data[name]

    def get_cum_data(self):
        return self._cum_data

    def update_state(self, st):
        self.update(st.launched, st.running, st.stale, st.note, st.monitor_msg)
            
    ##\brief Updates test record with current state
    ##
    ## Looks at current, previous state to record data and send alerts
    ##\param launched bool : Test launched
    ##\param running bool : Running (status OK)
    ##\param stale bool : Test is stale 
    ##\param note str : Notes from operator
    ##\param monitor_msg str : Message from Test Monitor
    ##\return (int, str) : int [0:OK, 1:Notify, 2:Alert]
    def update(self, launched, running, stale, note, monitor_msg):
        # Something wrong here, cum seconds not updating
        d_seconds = 0
        if self._was_running and running:
            d_seconds = rospy.get_time() - self._last_update_time

        self._cum_seconds += d_seconds

        alert = 0 # 0 - None, 1 - Notify, 2 - alert
        msg = ''
        state = 'Running'

        if launched and (not running) and stale:
            state = 'Stale'
        elif launched and (not running):
            state = 'Halted'
        elif not launched:
            state = 'Stopped'

        if (not self._was_launched) and launched:
            alert = 1
            msg = "Launched."
        elif self._was_launched and (not launched):
            alert = 1
            msg = "Shut down."

        elif self._was_running and (not running):
            alert = 2
            self._num_halts += 1
            if stale:
                msg = "Stale."
            else:
                msg = "Stopped."
        elif (not self._was_running) and running:
            alert = 1
            msg = "Restarted."

        if alert > 0:
            self._num_events += 1

        # Update cumulative parameters
        for param in self._test.params:
            if param.is_rate():
                self._cum_data[param.get_name()] += d_seconds * param.get_value()

        self._was_running = running
        self._was_launched = launched
        self._last_update_time = rospy.get_time()

        # Update with alert, note or every two hours of run time
        if alert > 0 or note != '' or  (running and self._last_log_time - rospy.get_time() > LOG_UPDATE):
            entry = LogEntry(rospy.get_time(), self.get_elapsed(), self.get_cum_time(), 
                             state, msg, self._test.params, self._num_halts, 
                             self._num_events, note)

            self._log_entries.append(entry)
            self._write_csv_entry(entry)
            self._last_log_time = self._last_update_time


        return (alert, msg)



    ##\brief Writes data to CSV, and to log entries
    def _write_csv_entry(self, entry):
        with open(self.log_file, 'ab') as f:
            log_csv = csv.writer(f)
             
            log_csv.writerow(entry.write_to_lst())

      
    def csv_filename(self):
        return self.log_file

    ##\brief Writes HTML table of last state of test
    ##
    ##\return str : HTML table 
    def write_table(self):
        html = '<table border="1" cellpadding="2" cellspacing="0">\n'
        time_str = format_localtime(self._start_time)
        html += write_table_row(['Start Time', time_str])
        html += write_table_row(['Elapsed Time', self.get_elapsed_str()])
        html += write_table_row(['Active Time', self.get_active_str()])
        for ky in self.get_cum_data().keys():
            cum_name = "Cum. %s" % ky
            html += write_table_row([cum_name, self.get_cum_data()[ky]])        

        html += write_table_row(['Num Halts', self._num_halts])
        html += write_table_row(['Num Alerts', self._num_events])
        html += '</table>\n'

        return html

    ##\brief Writes HTML table of test events and messages
    ##
    ##\return str : HTML table
    def write_summary_log(self):
        if len(self._log_entries) == 0:
            return '<p>No test log.</p>\n'

        html = []

        html.append('<table border="1" cellpadding="2" cellspacing="0">')
        html.append(write_table_row(['Time', 'Entry'], True))
        
        for entry in self._log_entries:
            time_str = format_localtime(entry.stamp)
            summary = entry.message + ' ' + entry.note

            html.append(write_table_row([time_str, summary]))
            
        html.append('</table>')

        return '\n'.join(html)

    ##\brief Writes full log to HTML table form
    def write_log(self):
        html = [ '<html>' ]

        html.append('<table border="1" cellpadding="2" cellspacing="0">')
        html.append(write_table_row(_get_csv_header_lst(self._test.params), True))

        for entry in self._log_entries:
            html.append(write_table_row(entry.write_to_lst()))

        html.append('</table>')

        html.append('</html>')

        return '\n'.join(html)
        
