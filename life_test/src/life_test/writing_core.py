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
##\brief Contains utilities for writing data to HTML/CSV output for logging

import math
import time

def write_table_row(lst, bold = False):
    """
    Writes list into HTML table row
    @param lst list : List of items to write
    @param bold bool : True if row text should be bold
    """
    html = ['<tr>']
    for val in lst:
        if bold:
            html.append('<td><b>%s</b></td>' % val)
        else:
            html.append('<td>%s</td>' % val)

    html.append('</tr>')

    return ''.join(html)


def get_duration_str(duration):
    """
    Returns duration in a formatted manner
    @param duration float : Time in seconds
    @return string "HOURShr, MINm"
    """
    hrs = max(math.floor(duration / 3600), 0)
    min = max(math.floor(duration / 6), 0) / 10 - hrs * 60
    
    return "%dhr, %.1fm" % (hrs, min)


def format_localtime(stamp):
    """
    Formats a rospy.get_time() stamp in a human readable local time
    @param stamp float : From rospy.get_time()
    """
    return time.strftime("%m/%d/%Y %H:%M:%S", time.localtime(stamp))
