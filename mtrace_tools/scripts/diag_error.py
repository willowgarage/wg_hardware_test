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
##\brief DiagnosticError 

PKG = 'mtrace_tools'
import roslib; roslib.load_manifest(PKG)
import sys 
import rospy
from collections import deque
import unittest
from itertools import izip


class BaseError(object):
    def __init__(self, name, t, desc):
        self.name = name     # compenent name
        self.t = t           # time
        self.desc = desc     # description of error
        self.sub_errors = [] # error caused by this error
        self.parents = []    # errors causing/contributing to this error
    def short_desc(self):
        return self.desc

class NoError(BaseError):
    """ Represents non-error, good for flushing error queues """
    def __init__(self, name, t, desc):
        BaseError.__init__(self, name, t, desc)

class GenericError(BaseError):
    """ Represents any type of unknown error """
    def __init__(self, name, t, desc):
        BaseError.__init__(self, name, t, desc)


class ErrorDelay:
    """ FIFO of error messages.  Errors are delayed by <delay> seconds.
    If reorder is True, will sort Errors using timestamps.
    If reorder is not True, expects input to be perfectly ordered.
    """
    def __init__(self, delay, reorder=False):
        self.delay = rospy.Duration.from_sec(delay)
        self.queue = deque()  #[]  #heapq()
        self.reorder = reorder
        self.begin_time = None

    def process(self, input_list):
        """ Takes input list of error messages and returns output list of delayed messages """
        if len(input_list) == 0:
            return []
        self.queue.extend(input_list)
        if self.reorder:
            if self.begin_time != None and (input_list[0].t < self.begin_time):
                raise Exception("Delay is not long enough to guarentee ordered output")
            self.queue = deque(sorted(self.queue, key=lambda error : error.t))
        queue = self.queue
        self.begin_time = queue[-1].t - self.delay
        # anything newer than begin_time can be returned
        output_list = []
        while len(queue) > 0:
            error = queue.popleft()
            if error.t < self.begin_time:
                output_list.append(error)
            else:
                queue.appendleft(error)
                break
        return output_list

    def all(self):
        """ Return all values and in queue """
        return list(self.queue)

    def peek(self):
        """ Returns first error in queue """
        return self.queue[0]


class TestErrorDelay(unittest.TestCase):
    # Test that ErrorDelay class delays error messages properly
    def setUp(self):
        pass

    def run_list(self, error_list, error_delay, delay):
        ros_delay = rospy.Duration.from_sec(delay)
        result_list = []
        for error in error_list:
            results = error_delay.process([error])
            len(results)
            result_list += results
            begin_time = error.t - ros_delay
            for error2 in result_list:
                self.assertTrue(begin_time >= error2.t)
            self.assertTrue(begin_time <= error_delay.peek().t, "%f, %f" % (begin_time.to_sec(), error_delay.peek().t.to_sec()))
        result_list += error_delay.all()
        self.verify_order(result_list)
        return result_list

    def run_list_in_steps(self, error_list, error_delay, delay, step_sizes):
        ros_delay = rospy.Duration.from_sec(delay)
        result_list = []
        start = 0
        for step in step_sizes:
            end = start+step+1
            slice = error_list[start:end]
            start = end
            results = error_delay.process(slice)
            if len(results) > 0:
                result_list += results
                begin_time = slice[-1].t - ros_delay
                for error2 in result_list:
                    self.assertTrue(begin_time >= error2.t)
                self.assertTrue(begin_time <= error_delay.peek().t, "%f, %f" % (begin_time.to_sec(), error_delay.peek().t.to_sec()))
        self.assertTrue(end>=len(error_list),'end=%d,len=%d'%(end,len(error_list)))
        result_list += error_delay.all()
        self.verify_order(result_list)
        return result_list

    # verify ordering of errors in list
    def verify_order(self, error_list):
        prev_error = error_list[0]
        for error in error_list:
            self.assertTrue(prev_error.t <= error.t, "%f, %f" % (prev_error.t.to_sec(), error.t.to_sec()))
            prev_error = error

    # verify every item in both lists match. 
    def verify_match(self, error_list, result_list):
        self.assertEqual(len(result_list), len(error_list), '%d, %d'%(len(result_list), len(error_list)))
        for index in range(len(result_list)):
            result, error = (result_list[index], error_list[index])
            self.assertEqual(result, error, 'i=%d, result[i]=%s, error[i]=%s'%(index,str(result),str(error)) )
        

    # Simple test that only inserts one new Error at a time
    def test_1_simple(self):
        # create list of errors equaly spaced out
        error_list = []
        for i in range(20):
            t = rospy.Time(i+10)
            error_list.append(GenericError('name', t, 'desc'))
        delay = 2.5
        error_delay = ErrorDelay(delay)
        result_list = self.run_list(error_list, error_delay, delay)
        self.verify_match(error_list, result_list)

    # Tests reordering capability of delay list
    def test_2_reorder(self):
        # create list of errors of semi-ordered items
        error_list = []
        for i in range(10):
            t = rospy.Time(i+10.5)
            error_list.append(GenericError('name', t, 'desc'))
            t = rospy.Time(i+10.0)
            error_list.append(GenericError('name', t, 'desc'))
        # ErrorDelay 
        delay = 2.5
        error_delay = ErrorDelay(delay, True)
        self.run_list(error_list, error_delay, delay)
        
    # Makes sure reordering throws error when delay is not large enough to reorder list
    def test_3_reorder(self):
        # create list of errors of semi-ordered items
        error_list = []
        for i in range(10):
            t = rospy.Time(i+10.5)
            error_list.append(GenericError('name', t, 'desc'))
            t = rospy.Time(i+10.0)
            error_list.append(GenericError('name', t, 'desc'))
        t = rospy.Time(10.0)
        error_list.append(GenericError('name', t, 'desc'))
        # ErrorDelay 
        delay = 2.5
        error_delay = ErrorDelay(delay, True)
        self.assertRaises(Exception, self.run_list, error_list, error_delay, delay)


    # Makes sure reordering returns empty list (not None) when there is nothing to return
    def test_4_empty(self):
        # create list of errors of semi-ordered items
        error_delay = ErrorDelay(10, True)
        error_list = error_delay.process([])
        result_type = type(error_list).__name__
        self.assertTrue(result_type == 'list', result_type)

    # Run test that inserts errors in different amounts
    def test_5_simple_steps(self):
        error_list = [ GenericError('name', rospy.Time(i+10), 'desc') for i in range(20) ]
        delay = 2.5
        error_delay = ErrorDelay(delay)
        steps = [i for i in range(len(error_list))]
        result_list = self.run_list_in_steps(error_list, error_delay, delay, steps)
        self.verify_match(error_list, result_list)

        

if __name__ == '__main__':
    unittest.main()
