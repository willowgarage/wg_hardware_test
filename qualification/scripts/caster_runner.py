#!/usr/bin/env python

from __future__ import division
PKG = 'qualification'
import roslib; roslib.load_manifest(PKG)

#import sys, os

import rospy

from std_msgs.msg import Float64

from sensor_msgs.msg import JointState

from pr2_self_test_msgs.srv import TestResultRequest

import numpy

CONTROLLER_NAME = 'caster_motor'
TOPIC_NAME = CONTROLLER_NAME + '/command'
JOINT_NAME = 'caster_motor_joint'

last_error = 10000000



# Return absolute error of velocity command
def get_error(msg, command):
    idx = -1
    for i, name in enumerate(msg.name):
        if name == JOINT_NAME:
            idx = i
            break

    global last_error
    last_error = abs(command - msg.velocity[idx])

def calc_mse(errors):
    errors_ar = numpy.array(errors)
    return numpy.dot(errors_ar, errors_ar) / (len(errors))

def pack_result(proxy, command, mse, tolerance):
    r = TestResultRequest()

    if mse < tolerance:
        r.result = TestResultRequest.RESULT_PASS
    else:
        r.result = TestResultRequest.RESULT_HUMAN_REQUIRED
    r.summary = 'Velocity command: %.2f. MSE: %.2f. Tolerance: %.2f' % (command, mse, tolerance)
    
    proxy.call(r)


if __name__ == '__main__':
    rospy.init_node('caster_runner')

    cmd_pub = rospy.Publisher(TOPIC_NAME, Float64)

    # Pull parameters
    cmd = Float64(rospy.get_param('~command', 16.5))
    timeout = rospy.get_param('~time', 15.0)
    tolerance = rospy.get_param('~tolerance', 1.0)

    result_srv = rospy.ServiceProxy('test_result', TestResultRequest)


    errors = [] # Store velocity tracking errors

    js_sub = rospy.Subscriber('joint_states', JointState, get_error, cmd.data)
                
    my_rate = rospy.Rate(10.0)
    start_time = rospy.get_time()
    end_time = start_time + timeout
    while not rospy.is_shutdown():
        if rospy.get_time() > end_time:
            break

        cmd_pub.publish(cmd)
        
        # Ignore tracking errors for the first few seconds
        if rospy.get_time() - start_time < 5.0:
            continue

        errors.append(last_error)
        my_rate.sleep()

    pack_result(result_srv, cmd.data, calc_mse(errors), tolerance)
        
    rospy.spin()
