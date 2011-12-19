#! /usr/bin/env python

PKG = 'mtrace_tools'

import roslib; roslib.load_manifest(PKG)
import rospy
from mtrace_tools.msg import MotorResistance
from pylab import *
from numpy import *
import numpy
import re
import rosrecord
import getopt
import math

class struct_of_arrays(object):
    def __init__(self, list_of_structs):
        # make sure type of input is correct
        #print list_of_structs
        #print type(list_of_structs)
        if type(list_of_structs).__name__ != 'list':
            print "object is not a list"
            return
        if len(list_of_structs) == 0:
            print "list is empty"
            return
        # pull in non __XXXX__ names, define them in this class as empty lists
        names = dir(list_of_structs[0])
        struct_names=[]
        for name in names:
            if name[0] != '_':
                struct_names.append(name)
                self.__setattr__(name,[])
        # Append values to lists
        for struct in list_of_structs:
            for name in struct_names:
                tmp = self.__getattribute__(name)
                tmp.append(struct.__getattribute__(name))
        # Convert lists into arrays        
        for name in struct_names:
            tmp = self.__getattribute__(name)
            array_tmp = array(tmp)
            self.__setattr__(name,array_tmp)


def callback(samples):
    #print time.strftime("Data was collected on %a, %d %Y %b %I:%M%a +0000", data.header.stamp)

    # assume samples is a list of structures
    min_current = 0.01
    no_zeros = []
    for s in samples:
        if abs(s.current) > min_current:
            no_zeros.append(s)
    a = struct_of_arrays(no_zeros)

    
    encoder_reduction = -1.0
    speed_constant = 158.0
    backemf_constant = 1.0 / (speed_constant * 2.0 * math.pi * 1.0/60.0);

    #float64 current
    #float64 voltage
    #float64 velocity
    #float64 position
    #bool    positive_direction
    backemf_voltage = a.velocity * encoder_reduction * backemf_constant 
    angular_position = mod(a.position,2.0 * math.pi)

    figure(1)
    resistance = (a.voltage-backemf_voltage) / a.current
    title("Resistance vs Position")
    plot(angular_position,resistance,'r.', label='Motor resitance vs position')

    figure(2)
    resistance_adj = backemf_voltage / a.current
    title("Backemf adjust resistance vs Position")
    plot(angular_position,resistance_adj,'b.', label='Back emf resitance vs position')

    figure(3)
    title("Current")
    plot(angular_position,a.current,'b.', label='current')

    show()




def usage():
    print "usage : resistance_plot.py filename"
    print "   filename - bag file with resistance measurements"

def mtrace_plot(argv):
    if len(argv) < 2:
        usage()
        return 1

    rospy.init_node("mtrace_plot", anonymous=True)

    bagfn = argv[1]
    samples = []
    for (topic, msg, t) in rosrecord.logplayer(bagfn, raw=False):
        samples += msg.samples
    
    print len(samples)
    callback(samples)


if __name__ == '__main__':    
   exit(mtrace_plot(sys.argv))




