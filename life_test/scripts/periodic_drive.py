#!/usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
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

# Author: Stuart Glaser
import time
import random
random.seed()
import roslib
roslib.load_manifest('life_test')
import rospy
from std_msgs.msg import Float64

from sensor_msgs.msg import JointState


STRAIGHT = 0.82
ROTATION_JOINT = 'fl_caster_rotation_joint'
SPEED = 10.0
PERIOD = 30.0
PI = 3.14159

class LastMessage(object):
    def __init__(self, topic, msg_type):
        self.msg = None
        rospy.Subscriber(topic, msg_type, self.callback)

    def last(self):
        return self.msg

    def callback(self, msg):
        self.msg = msg

def main():
    angle = STRAIGHT
    speed = -SPEED
    last_time = 0
    rospy.init_node('caster_cmder')
    last_state = LastMessage('joint_states', JointState)
    pub_steer = rospy.Publisher("caster_fl/steer", Float64)
    pub_drive = rospy.Publisher("caster_fl/drive", Float64)
    pub_steer.publish(Float64(0.0))
    pub_drive.publish(Float64(0.0))
    my_rate = rospy.Rate(100)

    rospy.loginfo("Waiting for a joint_state message...")
    while not last_state.msg and not rospy.is_shutdown(): my_rate.sleep()
    while not rospy.is_shutdown():
        my_rate.sleep()
        jnt_states = last_state.last()
        rotation_idx = -1
        for i, name in enumerate(jnt_states.name):
            if name == ROTATION_JOINT:
                rotation_idx = i
                break
        if rotation_idx < 0:
            rospy.logwarn("The %s joint was not found in the mechanism state" % ROTATION_JOINT)

        # Steers the caster to be straight
        angle_diff = angle - jnt_states.position[rotation_idx]
        pub_steer.publish(Float64(6.0 * angle_diff))

        # Drive
        if abs(angle_diff) < 0.05:
            pub_drive.publish(Float64(speed))
        else:
            pub_drive.publish(Float64(0.0))

        if rospy.get_time() - last_time > (PERIOD / 2):
            speed *= -1
            if(random.random() > 0.25): # Rotate caster
                rospy.loginfo('Rotating caster')
                pub_drive.publish(Float64(speed))
                time.sleep(0.75) # Allow for rotation
                speed *= -1
                if angle > PI:
                    angle -= PI
                else:
                    angle += PI
            last_time = rospy.get_time()


if __name__ == '__main__':
    main()
