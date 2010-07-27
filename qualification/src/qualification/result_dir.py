#!/usr/bin/env python

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
##\brief Checks that we can write to the temp directory for results

import os, tempfile

TEMP_DIR = os.path.join(tempfile.gettempdir(), 'qualification')
RESULTS_DIR = os.path.join(os.path.expanduser('~/wg_hardware_test'), 'qualification')

def check_qual_temp_dir():
    try:
        if not os.path.isdir(TEMP_DIR):
            os.mkdir(TEMP_DIR)
    except Exception, e:
        import traceback
        traceback.print_exc()
        return False

    return os.access(TEMP_DIR, os.R_OK) and os.access(TEMP_DIR, os.W_OK) and os.access(TEMP_DIR, os.X_OK)

def check_qual_result_dir():
    try:
        if not os.path.isdir(RESULTS_DIR):
            os.mkdir(RESULTS_DIR)
    except Exception, e:
        import traceback
        traceback.print_exc()
        return False

    return os.access(RESULTS_DIR, os.R_OK) and os.access(RESULTS_DIR, os.W_OK) and os.access(RESULTS_DIR, os.X_OK)
