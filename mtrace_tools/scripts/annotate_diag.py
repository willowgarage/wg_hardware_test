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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#


##\author Derek King
##\brief Annotates errors in diagnostic bag files.


PKG = 'mtrace_tools'

import roslib; roslib.load_manifest(PKG)

from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue

import rosrecord
import time
import sys 
import os
import re
from collections import deque
import unittest
import getopt
import rospy
import itertools

from diag_error import BaseError, GenericError, ErrorDelay, NoError
from delay_queue import DelayQueue


"""
usage: %(progname)s [-h] <bagfile>
  Goes through diagnostic <bagfile> and annotates errors
 
  Options:
  -h : show this help
"""

def usage(progname):
  print __doc__ % vars()


class ConvertVar:
    def __init__(self, var_name, convert_func, default):
        if var_name[0] == '_':
            raise Exception("Cannot use var_name begining with '_'")
        self.var_name = var_name
        self.convert_func = convert_func
        self.default = default

    def convert(self, obj, value):
        setattr(obj, self.var_name, self.convert_func(value))

    def set_default(self, obj):
        setattr(obj, self.var_name, self.default)

class ConvertList:
    def __init__(self, var_name, convert_func, index, default):
        if var_name[0] == '_':
            raise Exception("Cannot use var_name begining with '_'")
        self.var_name = var_name
        self.convert_func = convert_func
        self.index = index
        self.default = default
        
    def _set(self, obj, value):
        l = getattr(obj, self.var_name, [])
        if len(l) < (self.index+1):
            for i in range(self.index+1-len(l)):
                l.append(None)
        l[self.index] = self.default
        setattr(obj, self.var_name, l)

    def convert(self, obj, value):
        self._set(obj, self.convert_func(value))

    def set_default(self, obj):
        self._set(obj, self.default)

class VarStorage:
    """ Used to store variables """

class KeyValueList:
    def __init__(self):
        self._fields = {}

    def add( self, key_name, convert_obj):
        """ Add conversion to list """ 
        self._fields[key_name] = convert_obj

    def convert(self, msg, obj):
        """ Convert values contained in DiagnosticStatus message into variable in obj """
        for kv in msg.values:
            convert_obj = self._fields.get(kv.key)
            if convert_obj != None:
                convert_obj.convert(obj, kv.value)

    def set_defaults(self, obj):
        """ Sets default variable values in object """
        for convert_obj in self._fields.itervalues():
            convert_obj.set_default(obj)

class OnlyMotorHaltedError(BaseError):
    """ Represents error from master of just 'Motors Halted' """
    def __init__(self, name, t, desc):
        BaseError.__init__(self, name, t, desc)

class MasterDiag:
    """ Looks for errors in EtherCAT Master """
    def __init__(self, diag_map):
        self.name = 'EtherCAT Master'
        diag_map[self.name] = self

        self.level = 0 
        self.message = "OK"

        kvl = KeyValueList()
        kvl.add('Dropped Packets', ConvertVar('dropped_packets', int, 0))
        kvl.add('RX Late Packet', ConvertVar('late_packets', int, 0))
        self.kvl = kvl
                        
        self.old = VarStorage()
        kvl.set_defaults(self.old)

    def is_match(self, msg):
        """ Returns true if msg should handled by this component """
        return msg.name == self.name
    
    def process(self, msg, t):
        """ Returns array of error descriptions this message my have caused """
        error_list = []

        old = self.old
        new = VarStorage()
        self.kvl.convert(msg, new)

        if msg.level > self.level:
            if msg.level==2 and msg.message == 'Motors halted':
                error = OnlyMotorHaltedError(self.name, t, msg.message)
            else:
                error = GenericError(self.name, t, "transitioned into level %d : %s" % (msg.level, msg.message))
            error_list.append(error)
        elif msg.level != 0 and msg.message != self.message:
            error_list.append(GenericError(self.name, t, "message changed to %s" % (msg.message)))

        if new.dropped_packets != old.dropped_packets:
            error_list.append(GenericError(self.name, t, "dropped %d packets" % (new.dropped_packets - old.dropped_packets)))

        if new.late_packets != old.late_packets:                                 
            error_list.append(GenericError(self.name, t, "%d late packets" % (new.late_packets - old.late_packets)))

        self.level = msg.level
        self.message = msg.message

        self.old = new

        return error_list



class SafetyDisableError(BaseError):
    """ Represents any type of EtherCAT device safety disable """
    def __init__(self, name, t, desc):
        BaseError.__init__(self, name, t, desc)

class UndervoltageLockoutError(BaseError):
    """ Represents undervoltage lockout of EtherCAT device """
    def __init__(self, name, t, desc):
        BaseError.__init__(self, name, t, desc)


class SafetyDisableStatus:
    def __init__(self, str):
        self.undervoltage = str.find("UNDERVOLTAGE") != -1
        self.over_current = str.find("OVER_CURRENT") != -1
        self.board_overtemp = str.find("BOARD_OVER_TEMP") != -1
        self.bridge_overtemp = str.find("HBRIDGE_OVER_TEMP") != -1
        self.operational = str.find("OPERATIONAL") != -1
        self.watchdog = str.find("WATCHDOG") != -1
        self.disabled = str.find("DISABLED") != -1
        self.enabled = str.find("ENABLED") != -1
        self.str = str

        if (self.disabled == self.enabled):
            raise Exception("disabled and enabled both set in %s", str)

    def compare(self, old):
        """ Compares this safety disable status against old status.
        returns true if safety disable is set that was not in old.  
        """
        return (self.undervoltage and not old.undervoltage) or \
            (self.watchdog and not old.watchdog) or \
            (self.bridge_overtemp and not old.bridge_overtemp) or \
            (self.board_overtemp and not old.board_overtemp) or \
            (self.over_current and not old.over_current) or \
            (self.operational and not old.operational) or \
            (self.disabled and not old.disabled)

    def just_undervoltage(self):
        """ Returns true if safety disable status is just undervoltage """
        return self.undervoltage and not (self.over_current or self.board_overtemp or self.bridge_overtemp or self.operational or self.watchdog)
        
    def to_str(self):
        return self.str


def decode_safety_disable(value):
    """ Return safety disable status object """
    return SafetyDisableStatus(value)


class EthercatDeviceDiag:
    """ Looks for errors in a specific EtherCAT Device """
    def __init__(self, diag_map, name, num_ports, has_encoder):
        self.name = name
        diag_map[self.name] = self
        self.num_ports = num_ports
        self.has_encoder = has_encoder

        kvl = KeyValueList()
        kvl.add('Safety Disable Status Hold', ConvertVar('safety_disable_status_hold', decode_safety_disable, SafetyDisableStatus("ENABLED (00)")))
        kvl.add('Num encoder_errors', ConvertVar('encoder_errors', int, 0))
        for i in range(4):
            kvl.add('RX Error Port %d'%i, ConvertList('rx_error', int, i, 0))
            kvl.add('Lost Link Port %d'%i, ConvertList('lost_link', int, i, 0))
        self.kvl = kvl

        self.old = VarStorage()
        kvl.set_defaults(self.old)
        
    def process(self, msg, t):        
        error_list = []

        name = self.name
        old = self.old
        new = VarStorage()
        self.kvl.convert(msg, new)

        if self.has_encoder and new.encoder_errors != old.encoder_errors:
            error_list.append(GenericError(name, t, "%d new encoder errors %d" % (new.encoder_errors - old.encoder_errors)))

        if (new.safety_disable_status_hold.compare(old.safety_disable_status_hold)):
            if (new.safety_disable_status_hold.just_undervoltage()):
                error_list.append(UndervoltageLockoutError(name, t, "undervoltage lockout"))
            else:
                error_list.append(GenericError(name, t, "safety disable status changed to %s" % (new.safety_disable_status_hold.to_str())))
            
        num_ports = len(new.rx_error)
        if num_ports != self.num_ports:
            error_list.append(GenericError(name, t, "changing number of ports from %d to %d" % (self.num_ports, num_ports)))
            self.num_ports = num_ports

        for i in range(self.num_ports):
            if new.rx_error[i] != old.rx_error[i]:
                error_list.append(GenericError(name,t,"%d RX errors" % (new.rx_error[i] - old.rx_error[i])))
            if new.lost_link[i] != old.lost_link[i]:
                error_list.append(GenericError(name,t,"%d lost links" % (new.lost_link[i] - old.lost_link[i])))

        self.old = new

        return error_list


class EthercatDeviceAddDiag:
    """ Looks for EtherCAT devices that are not already present and adds a new EtherCAT Device Diag for them """

    def __init__(self, diag_list, diag_map):
        self.name = 'EthercatDeviceAddDiag'
        diag_list.append(self)

        self.diag_list = diag_list
        self.diag_map = diag_map

        self.is_ethercat_device = re.compile("EtherCAT Device \(\w+\)")

    def is_match(self, msg):
        m = self.is_ethercat_device.match(msg.name)
        return m != None
            
    def process(self, msg, t):
        name = msg.name
        #print "Found EtherCAT Device %s" % name

        # Use the HW id to figure number of ports and whether device has encoder
        num_ports = 1
        has_encoder = False
        if (re.match("68-05005-[0-9]{5}$", msg.hardware_id)):
            num_ports = 2
            has_encoder = True
        elif (re.match("68-05006-[0-9]{5}$", msg.hardware_id)):
            num_ports = 1
            has_encoder = True
        elif (re.match("68-05014-[0-9]{5}$", msg.hardware_id)):
            num_ports = 4
        elif (re.match("68-05021-[0-9]{5}$", msg.hardware_id)):
            num_ports = 2
        else:
            print "Don't understand hardware_id = ", msg.hardware_id

        dev = EthercatDeviceDiag(self.diag_map, name, num_ports, has_encoder)

        return dev.process(msg,t)


class RealtimeControlLoopDiag:
    """ Looks for issues occurring in 'Realtime Control Loop' """
    def __init__(self, diag_map):
        self.name = "Realtime Control Loop"
        diag_map[self.name] = self

        self.level = 0 
        self.message = "OK"

        kvl = KeyValueList()
        kvl.add('Control Loop Overruns', ConvertVar('control_loop_overruns', int, 0))
        kvl.add('Max EtherCAT roundtrip (us)', ConvertVar('max_ethercat_roundtrip', float, 0.0))
        kvl.add('Max Controller Manager roundtrip (us)', ConvertVar('max_controller_manager_roundtrip', float, 0.0))

        self.kvl = kvl
        self.old = VarStorage()
        kvl.set_defaults(self.old)
        
    def process(self, msg, t):        
        error_list = []

        name = self.name
        old = self.old
        new = VarStorage()
        self.kvl.convert(msg, new)

        #There are too many control loop overruns, don't print this out for now
        #if new.control_loop_overruns != old.control_loop_overruns:
        #    error_list.append("%d new control_loop_overruns" % (new.control_loop_overruns - old.control_loop_overruns))
            
        if (new.max_ethercat_roundtrip > old.max_ethercat_roundtrip) and (new.max_ethercat_roundtrip > 1000):
            error_list.append(GenericError(name, t, "Max ethercat roundtrip %f" % (new.max_ethercat_roundtrip)))

        if (new.max_controller_manager_roundtrip > old.max_controller_manager_roundtrip) and (new.max_controller_manager_roundtrip > 1000):
            error_list.append(GenericError(name, t, "Max controller roundtrip %f" % (new.max_controller_manager_roundtrip)))

        self.old = new
        self.level = msg.level
        self.message = msg.message

        return error_list

        
class RunStopError(BaseError):
    """ Represents Run-Stop being pressed. """
    def __init__(self, name, t, desc):
        BaseError.__init__(self, name, t, desc)
        self.undervoltage_errors = []
        self.motors_halted = None
    def short_desc(self):
        desc = self.desc
        desc += ' + motors halted' if (self.motors_halted != None) else ''
        if len(self.undervoltage_errors) > 0:
            desc += ' + ' + str(len(self.undervoltage_errors)) + ' undervoltage errors'
        return desc

def str_to_bool(str):
    if (str == "True"):
        return True
    elif (str == "False"):
        return False
    else:
        raise Exception("Not boolean : %s" % str)

class PowerBoardDiag:
    """ Looks for issues occurring in 'Power Board' """
    def __init__(self, diag_map, name):
        self.name = name
        diag_map[self.name] = self

        self.level = 0 
        self.message = "Running"

        kvl = KeyValueList()
        kvl.add('RunStop Button Status', ConvertVar('runstop_button_status', str_to_bool, True))
        kvl.add('RunStop Wireless Status', ConvertVar('runstop_wireless_status', str_to_bool, True))

        self.kvl = kvl
        self.old = VarStorage()
        kvl.set_defaults(self.old)
        
    def process(self, msg, t):        
        error_list = []

        name = self.name
        old = self.old
        new = VarStorage()
        self.kvl.convert(msg, new)

        if (not new.runstop_button_status and old.runstop_button_status) or \
                (not new.runstop_wireless_status and old.runstop_wireless_status):
            error_list.append(RunStopError(name, t, "Runstop"))

        self.old = new
        self.level = msg.level
        self.message = msg.message

        return error_list


class PowerBoardAddDiag:
    """ Looks for Power Board Devices and adds a PowerBoard for them """
    def __init__(self, diag_list, diag_map):
        self.name = 'PowerBoardAddDiag'
        diag_list.append(self)
        self.diag_list = diag_list
        self.diag_map = diag_map
        self.is_power_board = re.compile("Power board [0-9]{4}$")

    def is_match(self, msg):
        m = self.is_power_board.match(msg.name)
        return m != None
            
    def process(self, msg, t):
        name = msg.name
        #print "New Power Board : %s" % name
        dev = PowerBoardDiag(self.diag_map, name)
        return dev.process(msg,t)

class RunStopErrorMerge:
    """ Merges Undervoltage errors and MotorsHalted error into Runstop error """
    def __init__(self):
        def get_t(obj):
            return obj.t
        past = rospy.Duration.from_sec(2)
        future = rospy.Duration.from_sec(5)
        self.dq = DelayQueue(self.merge, get_t, future, past)

    def process(self,error_list):
        return self.dq.process(error_list)

    def merge(self, error, future, past):
        if type(error).__name__ != 'RunStopError':
            return
        for error2 in itertools.chain(future,past):
            typename = type(error2).__name__
            if typename == 'UndervoltageLockoutError':
                error.sub_errors.append(error2)
                error.undervoltage_errors.append(error2)
                error2.parents.append(error)
            elif typename == 'OnlyMotorHaltedError' and error.motors_halted == None:
                error.sub_errors.append(error2)
                error.motors_halted = error2
                error2.parents.append(error)

class PrintErrors:
    def __init__(self, duration):
        self.duration = duration
        self.last_time = rospy.Time(0)
        self.last_name = "NO_ERROR"

    def process(self, error_list):
        if len(error_list) == 0:
            return

        for error in error_list:
            if type(error).__name__ == 'NoError':
                continue
            if len(error.parents) > 0:
                continue
            if (error.t - self.last_time).to_sec() > self.duration:
                print "On %s" % time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(error.t.to_sec())) 
                #t1 = time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(self.last_time.to_sec()))
                #t2 = time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(error.t.to_sec())) 
                #print "Between %s and %s" % (t1,t2)
                self.last_time = error.t
                print ' ', error.name
                self.last_name = error.name

            if error.name != self.last_name:
                print ' ', error.name
                self.last_name = error.name
            print '   ', error.short_desc()


def main(argv):
    progname = argv[0]
    optlist, argv = getopt.getopt(argv[1:], "ht", ["help", "test"])
    for (opt, val) in optlist:
        if opt == "--help" or opt == '-h':
            usage(progname)
            return 0
        elif opt == "--test" or opt == '-t':
            return 0
        else:
            print "Internal error : unhandled option '%s'"%opt
            return 1

    inbag_filename = argv[0] 
    if not os.path.isfile(inbag_filename): 
        print >> sys.stderr, "Cannot locate input bag file [%s]" % inbag_filename 
        return 2

    diag_list = []
    diag_map = {}

    MasterDiag(diag_map)
    RealtimeControlLoopDiag(diag_map)
    EthercatDeviceAddDiag(diag_list, diag_map)
    PowerBoardAddDiag(diag_list, diag_map)

    # Use 30 second reordering buffer.
    reorder = ErrorDelay(30.0, True)    
    merge_list = []
    merge_list.append(RunStopErrorMerge())

    print_errors = PrintErrors(1.0)
    last_error_time = rospy.Time(0)


    for topic, msg, tbag in rosrecord.logplayer(inbag_filename):
        t = msg.header.stamp # use timestamp from diagnostic msg head instead of bag timestamp
        header = False
        error_list = []
        for status in msg.status:
            if status.name in diag_map:
                error_list += diag_map[status.name].process(status, t)
            else:
                for diag in diag_list:
                    if diag.is_match(status):
                        error_list += diag.process(status, t)                    
                        break  # stop searching when first match is found
        
        #if len(error_list) > 0:
        #    last_name = error_list[0].name
        #    print "On %s" % time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(t.to_sec())) 
        #    for error in error_list:
        #        if error.name != last_name:
        #            print ' ', error.name
        #            last_name = error.name
        #        print '  ', error.desc
        

        # Generate no-errors every 10seconds to keep merge queues moving
        if len(error_list) > 0:
            last_error_time = t
        elif (t-last_error_time).to_sec() > 10.0:
            #print "FLUSH ------------------------"
            error_list.append(NoError("flush", t, "10second flush"))
            last_error_time = t

        # Put messages into reordering queue
        error_list = reorder.process(error_list)

        len(error_list)

        # Process error list
        for merge in merge_list:
            error_list = merge.process(error_list)            

        # Process list
        print_errors.process(error_list)

        
    print "Log ends %s" % time.strftime("%a, %b %d, %I:%M:%S %p", time.localtime(t.to_sec()))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
