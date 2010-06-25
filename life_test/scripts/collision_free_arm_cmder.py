#! /usr/bin/env python
#
# Copyright (c) 2010, Willow Garage, Inc.
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

##\author Kevin Watts
##\brief Sends goals to arm to move it in collision free way

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib
import random
random.seed()

from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import GoalStatus

FAILED_OUT = 5

ranges = {
    'r_shoulder_pan_joint': (-2.0, 0.4),
    'r_shoulder_lift_joint': (-0.4, 1.25),
    'r_upper_arm_roll_joint': (-3.5, 0.3),
    'r_elbow_flex_joint': (-2.0, -0.05) 
}

recovery = {
    'r_shoulder_pan_joint': -0.8,
    'r_shoulder_lift_joint': 0.3,
    'r_upper_arm_roll_joint': 0.0,
    'r_elbow_flex_joint': -0.2
}

##\brief Moves arms to 0 position using unsafe trajectory
def get_recovery_goal():
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration.from_sec(5)    
    
    goal = JointTrajectoryGoal()
    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.get_rostime()

    for joint, pos in recovery.iteritems():
        goal.trajectory.joint_names.append(joint)
        goal.trajectory.points[0].positions.append(pos)
        goal.trajectory.points[0].velocities.append(0)

    return goal

if __name__ == '__main__':
    rospy.init_node('arm_cmder_client')
    client = actionlib.SimpleActionClient('collision_free_arm_trajectory_action_right_arm', 
                                          JointTrajectoryAction)
    rospy.loginfo('Waiting for server for collision free arm commander')
    client.wait_for_server()

    rospy.loginfo('Arm goals canceled')
    my_rate = rospy.Rate(1.0)

    recovery_client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action',
                                                JointTrajectoryAction)

    fail_count = 0
    while not rospy.is_shutdown():
        goal = JointTrajectoryGoal()
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(0)

        goal.trajectory.points.append(point)

        for joint, rng in ranges.iteritems():
            goal.trajectory.joint_names.append(joint)
                        
            goal.trajectory.points[0].positions.append(random.uniform(rng[0], rng[1]))
            
        rospy.logdebug('Sending goal to arm: %s' % str(point))
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(5))
        my_result = client.get_state()

        if my_result == GoalStatus.SUCCEEDED:
            fail_count = 0
        else:
            fail_count += 1

        if fail_count > FAILED_OUT:
            rospy.logwarn('Arm may be stuck. Sending recovery goal to free it')
            fail_count = 0
            client.cancel_goal()
            recovery_goal = get_recovery_goal()
            recovery_client.send_goal(recovery_goal)
            recovery_client.wait_for_result(rospy.Duration.from_sec(10))
            recovery_client.cancel_goal()

            
        my_rate.sleep()
