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

from __future__ import with_statement, division

PKG = 'life_test'
import roslib
roslib.load_manifest(PKG)

import csv
import os, sys, math

import rospy

from socket import gethostname

from writing_core import *

import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import Encoders

import tempfile, tarfile

import wg_invent_client

LOG_UPDATE = 7200 # Update log if no entries before this time
INVENT_TIMEOUT = 600

import result_dir
RESULTS_DIR = result_dir.RESULTS_DIR


def _get_csv_header_lst(params):
    """
    Returns header to a CSV log file as a list
    @param params [ TestParam ] : Test parameters
    @return [ str ] : Header items
    """
    hdr = [ 'Time', 'Status', 'Message', 'Monitor Status', 'Elapsed', 
            'Cum. Time', 'Num. Halts', 'Num. Events' ]

    for p in params:
        if p.rate:
            hdr.append('Cum. %s' % p.name)

    hdr.append('Note')

    return hdr

def _alert_prefix(lvl):
    """
    Return email subject prefix based on status level
    """
    if lvl == 2:
        return '--Test Alert--'
    if lvl > 2:
        return '--Test Report--'
    return '--Test Notify--'

class LogEntry(object):
    """
    Entry in test log. Gets written to CSV output file.
    """
    def __init__(self, stamp, elapsed, cum_time, status, message, monitor_msg, params, halts, events, note):
        """
        @param stamp float : Timestamp
        @param elapsed float : Seconds elapsed since start
        @param cum_time float : Cumulative (running) time
        @param status str : Status string
        @param message str : Message from manager
        @param monitor_msg str : Message from test monitor
        @param params [ TestParam ] : Values of parameters
        @param halts int : Number of halts
        @param events int : Number of events
        @param note str : Note from user
        """
        self.stamp = stamp

        self.status = status
        self.message = message
        self.monitor_msg = monitor_msg
        self.params = params
        self.note = note

        # Time in seconds
        self.elapsed = elapsed
        self.cum_time = cum_time

        self.halts = halts
        self.events = events

    def write_to_lst(self):
        """
        Write data to list, to be written to CSV
        @return [ str ] : List of all items
        """
        lst = [ format_localtime(self.stamp), self.status, self.message, self.monitor_msg,
                get_duration_str(self.elapsed), 
                get_duration_str(self.cum_time), self.halts, self.events ]
 
        for p in self.params:
            if p.rate:
                lst.append(p.value * self.cum_time)

        lst.append(self.note)

        return lst

    
def _get_hrs(seconds):
    """
    Converts seconds to hours to 0.1 hours precision
    @param seconds : Seconds 
    @return float : Hours to 0.1 decimal places
    """
    hrsx10 = int(seconds / 360)
    return hrsx10 / 10.

class TestRecord(object):
    """
    Updates CSV record with state changes for a test    
    
    """
    def __init__(self, test, serial, file_path = None, csv_name = None, send_email = True):
        """
        @param test LifeTest : Test type, params
        @param serial str : Serial number of DUT
        @param file_path str : File path of out. Used for unit testing.
        """
        self._start_time = rospy.get_time()
        self._cum_seconds = 0
        self._last_update_time = rospy.get_time()

        # Last state of test
        self._was_running = False
        self._was_launched = False
        self._last_msg = ''

        self._num_events = 0
        self._num_halts = 0
        self._test_complete = False
        self._bay = None

        self._serial = serial
        self._test = test

        self._log_entries = []

        self._last_log_time = self._last_update_time
        
        self._last_invent_time = 0
        self._invent_note_id = None

        self._cum_data = {}
        for param in test.params:
            if param.rate:
                self._cum_data[param.name] = 0

        if not csv_name:
            csv_name = str(self._serial) + '_' + format_localtime_file(self._start_time) + \
                '_' + self._test.name + '.csv'
        if not csv_name.endswith('.csv'):
            csv_name += '.csv'

        csv_name = csv_name.replace(' ', '_').replace('/', '-')

        if not file_path:
            file_path = RESULTS_DIR
        else:
            file_path = os.path.expanduser(file_path)

        if not os.path.isdir(file_path):
            os.makedirs(file_path)

        self.log_file = os.path.join(file_path, csv_name)

        # Write header if file doesn't already exist
        if not os.path.exists(self.log_file):
            with open(self.log_file, 'ab') as f:
                log_csv = csv.writer(f)
                log_csv.writerow(_get_csv_header_lst(self._test.params))

        self._has_checked_invent = False
        self._invent_hrs_base = 0.0

        self._send_email = send_email


    def get_elapsed(self):
        """
        Time since test was started, in seconds.
        """
        elapsed = rospy.get_time() - self._start_time
        return elapsed

    def get_cum_time(self):
        """
        Time that test has been running, in seconds.
        """
        return self._cum_seconds

    def get_active_str(self):
        return get_duration_str(self._cum_seconds)

    def get_elapsed_str(self):
         return get_duration_str(self.get_elapsed())
            
    def update(self, launched, running, stale, note, monitor_msg):
        """
        Updates test record with current state
        
        Looks at current, previous state to record data and send alerts
        \param launched bool : Test launched
        \param running bool : Running (status OK)
        \param stale bool : Test is stale 
        \param note str : Notes from operator
        \param monitor_msg str : Message from Test Monitor
        \return (int, str) : int [0:OK, 1:Notify, 2:Alert]
        """
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
            if param.rate:
                self._cum_data[param.name] += d_seconds * param.value

        self._was_running = running
        self._was_launched = launched
        self._last_msg = monitor_msg
        self._last_update_time = rospy.get_time()

        # Update with alert, note or every two hours of run time
        if alert > 0 or note != '' or  (running and self._last_log_time - rospy.get_time() > LOG_UPDATE):
            entry = LogEntry(rospy.get_time(), self.get_elapsed(), self.get_cum_time(), 
                             state, msg, monitor_msg, self._test.params, self._num_halts, 
                             self._num_events, note)

            self._log_entries.append(entry)
            self._write_csv_entry(entry)
            self._last_log_time = self._last_update_time

        # Email operator with details
        if alert > 0:
            self.notify_operator(alert, msg)

        return (alert, msg)

    def _write_csv_entry(self, entry):
        """
        Writes data to CSV, and to log entries
        """
        with open(self.log_file, 'ab') as f:
            log_csv = csv.writer(f)
             
            log_csv.writerow(entry.write_to_lst())

    ##\todo property
    def csv_filename(self):
        return self.log_file

    def _get_test_team(self):
        # HACK!!! Don't email everyone if it's debugging on NSF
        if os.environ.has_key('USER') and os.environ['USER'] == 'watts' and gethostname() == 'nsf':
            return 'watts@willowgarage.com'

        return 'test.team@lists.willowgarage.com'

    def _email_subject(self, lvl, msg):
        return _alert_prefix(lvl) + " Test %s. MSG: %s" % (self._test.get_title(self._serial), msg)
        
    def make_email_message(self, level, alert_msg = ''):
        """
        Called during unit testing and operator notificaton
        """
        msg = MIMEMultipart('alternative')
        msg['Subject'] = self._email_subject(level, alert_msg)
        msg['From'] = 'test.notify@willowgarage.com'
        msg['To'] = self._get_test_team()
        
        msg.attach(MIMEText(self.make_html_test_summary(alert_msg), 'html'))
        
        log_csv = open(self.log_file, 'rb')
        log_data = log_csv.read()
        log_csv.close()
        
        part = MIMEBase('application', 'octet-stream')
        part.set_payload(log_data)
        Encoders.encode_base64(part)
        part.add_header('Content-Disposition', 'attachment; filename="%s"' 
                        % os.path.basename(self.csv_filename()))
        
        msg.attach(part)

        return msg

    def notify_operator(self, lvl, alert_msg):
        """
        Sends email with test info to testing team
        """
        # Don't send email if we have that disabled
        if not self._send_email:
            return True

        try:
            msg = self.make_email_message(lvl, alert_msg)

            s = smtplib.SMTP('localhost')
            s.sendmail(msg['From'], msg['To'], msg.as_string())
            s.quit()

            return True
        except Exception, e:
            import traceback
            rospy.logwarn('Unable to send mail! %s' % traceback.format_exc())
            return False

    def complete_test(self):
        """Test is marked as finished."""
        self._test_complete = True
        
    @property
    def test_complete(self):
        return self._test_complete

    def set_bay(self, bay):
        """
        The bay the test runs on. Can only be called when test isn't launched.
        """
        self._bay = bay
        
        if not bay:
            return
        
        entry = LogEntry(rospy.get_time(), self.get_elapsed(), self.get_cum_time(), 
                 'Stopped', '', '', self._test.params, 
                 self._num_halts, self._num_events, 
                 'Test bay: %s. Power board, breaker: %s, %s' % 
                 (self._bay.name, self._bay.board, self._bay.breaker))

        self._log_entries.append(entry)
        self._write_csv_entry(entry)
        
    def make_html_test_summary(self, alert_msg = None):
        html = ['<html><head><title>Test Log: %s of %s</title>' % (self._test.name, self._serial)]
        html.append('<style type=\"text/css\">\
body { color: black; background: white; }\
div.error { background: red; padding: 0.5em; border: none; }\
div.warn { background: orange: padding: 0.5em; border: none; }\
div.pass { background: green; padding: 0.5em; border: none; }\
strong { font-weight: bold; color: red; }\
em { font-style:normal; font-weight: bold; }\
</style>\
</head>\n<body>')

        html.append('<H2 align=center>Test Log: %s of %s</H2>' % (self._test.name, self._serial))
        
        if alert_msg:
            html.append('<H3>Alert: %s</H3><br>' % alert_msg)

        if self._test_complete:
            html.append('<H3>Test Complete</H3>')
        else:
            if self._was_launched and not self._was_running:
                html.append('<H3>Test Status: Launched, Halted</H3>')
            elif self._was_launched and self._was_running:
                html.append('<H3>Test Status: Launched, Running</H3>')
            else:
                html.append('<H3>Test Status: Shutdown</H3>')

        html.append('<H4>Current Message: %s</H4>' % str(self._last_msg))

        # Table of test bay, etc
        html.append('<hr size="3">')
        html.append('<H4>Test Info</H4>')
        html.append('<p>Description: %s</p><br>' % self._test.desc)
        html.append(self._make_test_info_table())

        # Parameter table
        html.append('<hr size="3">')
        html.append('<H4>Test Parameters</H4>')
        html.append(self._test.make_param_table())

        # Make final results table
        html.append('<hr size="3">')
        html.append('<H4>Test Results</H4>')
        html.append(self._write_table())
        
        # Make log table
        html.append('<hr size="3">')
        html.append('<H4>Test Log</H4>')
        html.append(self._write_summary_log())

        html.append('<hr size="3">')
        html.append('</body></html>')

        return '\n'.join(html)

    def _make_test_info_table(self):
        """
        Writes HTML table of test info
        @return str : HTML table   
        """
        html = ['<table border="1" cellpadding="2" cellspacing="0">']
        html.append(write_table_row(['Test Name', self._test.name]))
        if self._bay:
            html.append(write_table_row(['Test Bay', self._bay.name]))
            html.append(write_table_row(['Machine', self._bay.machine]))
            html.append(write_table_row(['Powerboard', self._bay.board]))
            html.append(write_table_row(['Breaker', self._bay.breaker]))

        
        html.append(write_table_row(['Serial', self._serial]))
        html.append(write_table_row(['Test Type', self._test.type]))
        html.append(write_table_row(['Launch File', self._test.launch_file]))
        html.append('</table>')

        return '\n'.join(html)

    def _write_table(self):
        """
        Writes HTML table of last state of test
        @return str : HTML table   
        """
        html = ['<table border="1" cellpadding="2" cellspacing="0">']
        time_str = format_localtime(self._start_time)
        html.append(write_table_row(['Start Time', time_str]))
        html.append(write_table_row(['Elapsed Time', self.get_elapsed_str()]))
        html.append(write_table_row(['Active Time', self.get_active_str()]))
        for ky in self._cum_data.keys():
            cum_name = "Cum. %s" % ky
            html.append(write_table_row([cum_name, self._cum_data[ky]]))  

        html.append(write_table_row(['Num Halts', self._num_halts]))
        html.append(write_table_row(['Num Alerts', self._num_events]))
        html.append('</table>')

        return '\n'.join(html)

    def _write_summary_log(self):
        """
        Writes HTML table of test events and messages
        @return str : HTML table
        """
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

    ##\brief 
    def write_log(self):
        """
        Writes full log to HTML table form
        @return str : Log of test
        """
        html = [ '<html>' ]

        html.append('<table border="1" cellpadding="2" cellspacing="0">')
        html.append(write_table_row(_get_csv_header_lst(self._test.params), True))

        for entry in self._log_entries:
            html.append(write_table_row(entry.write_to_lst()))

        html.append('</table>')

        html.append('</html>')

        return '\n'.join(html)

    def write_tar_file(self):
        """
        Writes CSV, HTML summary into tar file.
        Unit testing and loading attachments
        
        ##\return Named temporary file. ".close()" will delete file
        """
        html_logfile = tempfile.NamedTemporaryFile()
        f = open(html_logfile.name, 'w')
        f.write(self.make_html_test_summary())
        f.close()
                

        tar_filename = tempfile.NamedTemporaryFile()

        tf = tarfile.open(tar_filename.name, 'w:')
        summary_name = self._serial + '_' + self._test.name.replace(' ', '_').replace('/', '-') \
            + '_summary.html'
        tf.add(html_logfile.name, arcname=summary_name)
        tf.add(self.csv_filename(), arcname=os.path.basename(self.csv_filename()))

        tf.close()

        html_logfile.close() # Deletes html logfile

        return tar_filename

    @property
    def result(self): 
        if self._test_complete: return 'Pass'
        return 'Not Finished'

    def _check_invent_hours(self, iv):
        """
        Checks Invent for  the number of hours that the test has run, 
        from Invent. Invent stores this data as a Key-Value for the item.
        Updates self._invent_hrs_base with data
        """
        if self._has_checked_invent:
            return

        if not iv.login():
            return

        hrs_str = iv.getKV(self._serial, self._test.id + ' Hours')
        if not hrs_str:
            self._has_checked_invent = True
            return

        try:
            self._invent_hrs_base = float(hrs_str)
        except ValueError, e:
            print >> sys.stderr, "Unable to recover Invent hours from %s, got %s" % (self._serial, hrs_str)
            self._has_checked_invent = True


    def _load_attachments(self, iv):
        """
        Load attachments to Invent
        @raise Exception : Exception from Invent 
        @return bool : True if loaded successfully
        """
        hrs_str = self.get_active_str()
        note = "%s finished. Total active time: %s." % (self._test.name, hrs_str)
        
        tfile = self.write_tar_file()
        f = open(tfile.name, 'rb')
        tr = f.read()
        f.close()

        archive_name = self._serial + '_' + self._test.name.replace(' ', '_').replace('/', '-') + '.tar'

        my_data = wg_invent_client.TestData(self._test.id, self._test.name, self._start_time, self._serial, self.result)

        my_data.set_note(note)
        my_data.set_attachment('application/tar', archive_name)

        rv = wg_invent_client.submit_log(iv, my_data, tr)

        tfile.close() # Deletes tar file

        return rv
        
        
    def load_attachments(self, iv):
        """
        Load attachment into inventory system as a tarfile
        Loads data as "test".

        @param iv Invent : Invent client, to load 
        @return bool : True if loaded successfully
        """
        try:
            if self.get_cum_time() == 0:
                return True # Don't log anything if didn't run

            self.update_invent(iv)

            return self._load_attachments(iv)
        except Exception, e:
            import traceback
            rospy.logerr('Unable to submit to invent. %s' % traceback.format_exc())
            return False

    def _get_total_hours(self, iv):
        """
        Returns string of total hours that this device has run under this
        test. Hours from previous tests are pulled from Invent. 

        \return str : Hours, to 0.1 hour precision. Ex: "10.2"
        """
        self._check_invent_hours(iv)
        return str(_get_hrs(self._cum_seconds + self._invent_hrs_base * 3600.))
        

    def update_invent(self, iv):
        """
        Update inventory system note with status, and update Key-Value
        with cumulative hours.

        @param iv Invent : Invent client, to load note
        """
        if rospy.get_time() - self._last_invent_time < INVENT_TIMEOUT:
            return

        # Don't log anything if we haven't launched
        if self.get_elapsed() == 0 and not self._invent_note_id:
            return

        self._last_invent_time = rospy.get_time()

        total_hours = self._get_total_hours(iv)
        iv.setKV(self._serial, self._test.id + ' Hours', 
                 total_hours)

        hrs_str = self.get_active_str()

        stats = "Stats: Total active time %s." % (hrs_str)
        
        if self._was_launched and self._was_running:
            note = "Test running: %s. " % (self._test.name)
        elif self._was_launched and not self._was_running:
            note = "%s is paused. " % self._test.name
        else:
            note = "%s stopped. " % self._test.name

        self._invent_note_id = iv.setNote(self._serial, note + stats, self._invent_note_id)
