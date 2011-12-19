#! /usr/bin/env python

PKG = 'mtrace_tools'

import roslib; roslib.load_manifest(PKG)
import rospy
from ethercat_hardware.msg import MotorTrace
from pylab import *
from numpy import *
import numpy
import re
import rosbag
import getopt
import time

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


def pretty_duration(duration):    
    in_future = False 
    if (duration < 0):
        duration = -duration
        in_future = True

    secs_per_min  = 60.0
    secs_per_hour = secs_per_min * 60.0
    secs_per_day  = secs_per_hour * 24.0
    secs_per_week = secs_per_day * 7.0

    weeks = math.floor(duration / secs_per_week)
    duration -= weeks * secs_per_week
    days = math.floor(duration / secs_per_day)
    duration -= days * secs_per_day
    hours = math.floor(duration / secs_per_hour)
    duration -= hours * secs_per_hour
    mins = math.floor(duration / secs_per_min)
    duration -= mins * secs_per_min
    result = ""
    if weeks > 0:    
        result += ("%d week%s "%(weeks, "s" if weeks > 1 else ""))
    if days > 0:    
        result += ("%d day%s "%(days, "s" if days > 1 else ""))
    if hours > 0:
        result += ("%d hour%s "%(hours, "s" if hours > 1 else ""))
    if mins > 0:
        result += ("%d minute%s "%(mins, "s" if mins > 1 else ""))
    if len(result) > 0:
        result += "and "
    result += "%s seconds "%duration
    if in_future:
        result += "in the future"
    else:
        result += "ago"
    return result

def callback(data):
    print "Got data for actuator %s with %d samples" % (data.actuator_info.name, len(data.samples))
    #print "Data was published %f seconds ago" % (rospy.get_rostime() - data.header.stamp).to_sec()
    print "Reason : %s" % data.reason
    print "Data was published : %s -- %s" % (time.strftime("%a, %b %d, %I:%M %p", time.localtime(data.header.stamp.to_sec())), pretty_duration((rospy.get_rostime() - data.header.stamp).to_sec()))
    print "Timestamp seconds = %d " % data.header.stamp.secs
    size = len(data.samples)
    if size <= 1 :    
        return

    #print type(data)
    #print type(data.samples)
    a = struct_of_arrays(data.samples)

    # Start time from 0
    t                = a.timestamp - a.timestamp[0]
    measured_motor_voltage = a.measured_motor_voltage
    supply_voltage   = a.supply_voltage
    measured_current = a.measured_current
    executed_current = a.executed_current
    programmed_pwm   = a.programmed_pwm
    velocity         = a.velocity
    encoder_position = a.encoder_position
    encoder_errors   = a.encoder_error_count

    filtered_voltage_error     = a.filtered_motor_voltage_error
    filtered_abs_voltage_error = a.filtered_abs_motor_voltage_error
    filtered_current_error     = a.filtered_current_error
    filtered_abs_current_error = a.filtered_abs_current_error
 
    actuator_info = data.actuator_info
    board_info = data.board_info

    M_PI = math.pi
    backemf_constant = 1.0 / (actuator_info.speed_constant * 2.0 * M_PI * 1.0/60.0);
    resistance = actuator_info.motor_resistance
    board_resistance = board_info.board_resistance
    print data.board_info
    print actuator_info
    print "backemf_constant: %f" % backemf_constant

    # Estimate motor resistance and backemf constant from data + allow voltage offset
    A = numpy.array([measured_current, velocity, numpy.ones(len(velocity))])
    x = numpy.linalg.lstsq(A.transpose(), measured_motor_voltage)
    print "ESTIMATES: "
    print "resistance: %f"%x[0][0]
    print "backemf_constant: %f"%-x[0][1]
    print "voltage_offset: %f"%x[0][2]
    print "END OF ESTIMATES"
    #backemf_constant = -x[0][1]
    #resistance = x[0][0]
    #voltage_offset = x[0][2]

    # Estimate motor resistance data + use fixed back emf constant + don't allow voltage offset
    #A = numpy.array([measured_current])
    #B = measured_motor_voltage - velocity * actuator_info.encoder_reduction * backemf_constant
    #x = numpy.linalg.lstsq(A.transpose(), B)
    #print "ESTIMATES: "
    #print "resistance: %f"%x[0][0]
    #print "END OF ESTIMATES"
    #resistance = x[0][0]

    # Estimate board resistance (resistance of MOSFETs and inductors)
    A = numpy.array([measured_current, numpy.ones(len(measured_current))])
    B = supply_voltage * programmed_pwm - measured_motor_voltage;  # 
    x = numpy.linalg.lstsq(A.transpose(), B)
    print "ESTIMATES(2):"
    print "board_resistance: %f"%x[0][0]
    print "voltage offset: %f"%x[0][1]
    print "END OF ESTIMATES(2)"
    #board_resistance = x[0][0]

    board_output_voltage  = supply_voltage * programmed_pwm - board_resistance * measured_current
    backemf_voltage = velocity * actuator_info.encoder_reduction * backemf_constant 
    resistance_voltage = resistance * measured_current
    motor_model_voltage = backemf_voltage + resistance_voltage    
 
    # Make cycle-by-cycle estimate of motor resistance    
    est_resistance = zeros(len(measured_current))
    est_resistance_accuracy = zeros(len(measured_current))
    # Don't calculate reistance if there is not enough motor current
    min_current_for_est = 0.02 * board_info.hw_max_current + 0.005
    for i in range(0,len(measured_current)-1):
        if (abs(measured_current[i]) > min_current_for_est):
            est_resistance[i] = (board_output_voltage[i] - backemf_voltage[i]) / measured_current[i]
            est_resistance_accuracy[i] = 1.0 / (1.0 + abs(backemf_voltage[i] / resistance_voltage[i]))


    figure(1)
    subplot(211)
    title("Motor Voltage")
    plot(t,measured_motor_voltage,'r-', label='Measured Motor (by ADC on board)')
    plot(t,board_output_voltage,'g-',   label='Board Output (PWM * Vsupply - R_brd*I)')
    plot(t,motor_model_voltage,'b-',    label='Motor Model (backEMF + R*I)')
    legend(loc='best')
    subplot(212)
    title("Motor Voltage Error")
    plot(t,measured_motor_voltage - board_output_voltage,'r-', label='Measured Motor - Board Output')
    plot(t,motor_model_voltage    - board_output_voltage,'b-', label='Motor Model    - Board Output')
    plot(t,a.motor_voltage_error_limit,                  'k-', label='Motor Voltage Error Limit')
    plot(t,-1 * a.motor_voltage_error_limit,                  'k-', label='__nolegend__')
    legend(loc='best')
    figure(5)
    title("Motor Voltage Relative Error");
    plot(t,filtered_voltage_error,    'r-',  label='Trace : Motor vs Board Error (Filter)')
    plot(t,filtered_abs_voltage_error,'g-',  label='Trace : Motor vs Board Error (Abs Filter)')
    legend(loc='best')
    figure(2)
    subplot(211)
    title("Motor Current")
    plot(t,measured_current,'b-', label='Measured')
    plot(t,executed_current,'r-', label='Executed')
    legend(loc='best')
    subplot(212)
    title("Motor Current Error")
    plot(t,measured_current-executed_current,'b-', label='Measured-Executed')
    plot(t,filtered_current_error,'r-',            label='Trace : Error (Filter)')
    plot(t,filtered_abs_current_error,'g-',        label='Trace : Error (Abs Filter)')
    legend(loc='best')
    figure(3)
    subplot(221)
    title("Supply Voltage")
    plot(t,supply_voltage)
    subplot(222)
    title("Back EMF (Velocity * Vsupply)")
    plot(t,backemf_voltage,'b-', label='Back EMF')
    subplot(223)
    title("Resistance * I")
    plot(t,resistance_voltage,'g-', label='Resistance * I')
    subplot(224)
    title("PWM %")
    plot(t,programmed_pwm * 100.0,'g-', label='PWM%')
    plot([t[0], t[-1]],ones(2) * -board_info.max_pwm_ratio * 100.0,'r-.', label='Min')
    plot([t[0], t[-1]],ones(2) * board_info.max_pwm_ratio * 100.0,'r-.', label='Max')

    figure(4)
    angular_position = mod(encoder_position,2.0 * M_PI)
    subplot(211)    
    title("Voltage Error vs Position")
    plot( angular_position, (motor_model_voltage - board_output_voltage) , "r.", label="Voltage Error")
    subplot(212)
    title("Current Error vs Position")
    plot( angular_position, (measured_current - executed_current) , "b.", label="Current")

    figure(6) 
    subplot(211)    
    title("Motor Resistance")
    plot( t, est_resistance , "r.", label="Estimated Resistance")
    plot( [t[0], t[-1]], ones(2) * resistance , "g-", label="Resistance")
    legend(loc='best')
    subplot(212)
    title("Resistance Accuracy")
    plot( t, est_resistance_accuracy , "g-", label="Accuracy")

    figure(7) 
    subplot(311)    
    title("Angular Position")
    plot( t, angular_position , "r-", label="Angualar position (0 to 2*PI)")
    legend(loc='best')
    subplot(312)
    title("Position")
    plot( t, encoder_position , "b-", label="Position")
    legend(loc='best')
    subplot(313)
    title("Velocity")
    plot( t, velocity , "g-", label="Velocity")
    legend(loc='best')

    figure(8)
    title("Velocity")
    plot( t, velocity , "g-", label="Velocity")
    legend(loc='best')


    show()

def read_bag(bagfn):
    bag = rosbag.Bag(bagfn)
    for topic, msg, t in bag.read_messages():
        callback(msg)


def _succeed(args):
    code, msg, val = args
    if code != 1:
        raise ROSTopicException("remote call failed: %s"%msg)
    return val

def usage():
    print "usage : old_mtrace_plot.py [-f filename] topic [more topics]"
    print "   topic - Regular express of topic name to subscribe to"
    print "   -f filename - bag file to read messages from"
    print "example : mtrace_plot.py l_wrist_r_motor"
    print "example : mtrace_plot.py -f pr1006_r_wrist_r_motor_error_2010-05-10-14-53-50.bag"

def mtrace_plot(argv):
    if len(argv) < 2:
        usage()
        return 1

    rospy.init_node("mtrace_plot", anonymous=True)

    optlist,argv = getopt.gnu_getopt(argv, "hf:");
    for opt,arg in optlist:
        if (opt == "-f"):
            read_bag(arg)
        elif (opt == "-h"):
            usage()
            return 0
        else :
            print "Internal error : opt = ", opt
            return 1
        
    if len(argv) > 1:
        master = roslib.scriptutil.get_master()
        pub_topics = _succeed(master.getPublishedTopics('/rostopic', '/'))
        valid_topics = []
        for topic,topic_type in pub_topics:
            if topic_type == 'ethercat_hardware/MotorTrace':
                valid_topics.append(topic)

        if len(valid_topics) == 0:
            print "Error: no valid topics being published"
            return 1

        topic_list = []
        for arg in sys.argv[1:]:
           regex = re.compile(arg)
           for topic in valid_topics:
               if regex.search(topic):
                   topic_list.append(topic)
        if len(topic_list) == 0:
           print "Error no topics names match"
           print "Possible topics : %s"%",".join(valid_topics)
           return 1

        #print ",".join(topic_list)

        for topic in topic_list:
            print "subscribing to topic %s" % topic
            rospy.Subscriber(topic, MotorTrace, callback)
        rospy.spin()

if __name__ == '__main__':    
   exit(mtrace_plot(sys.argv))




