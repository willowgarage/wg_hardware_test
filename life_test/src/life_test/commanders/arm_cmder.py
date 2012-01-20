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
##\brief Sends goals to arms in collision free way.

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib
import random
random.seed()

from pr2_controllers_msgs.msg import JointTrajectoryGoal, JointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib_msgs.msg import GoalStatus

FAILED_OUT = 5

##\brief Moves arms to 0 position using unsafe trajectory
##
##\param recovery { str : float } : Recover positions for arms
##\return JointTrajectoryGoal for arm 
def _get_recovery_goal(recovery):
    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration.from_sec(5)    
    
    goal = JointTrajectoryGoal()
    goal.trajectory.points.append(point)
    goal.trajectory.header.stamp = rospy.get_rostime()

    for joint, pos in recovery.iteritems():
        goal.trajectory.joint_names.append(joint)
        goal.trajectory.points[0].positions.append(pos)
        goal.trajectory.points[0].velocities.append(0.0)

    return goal

class ArmCmder(object):
    """
    This class sends commands random to an arm using a JointTrajectoryAction. 
    It sends "safe" commands by default. If the arm fails to move after a few tries, 
    it sends an unsafe, predefined recovery command.
    """
    def __init__(self, arm_client, ranges, recovery_client, recovery_positions):
        """
        \param arm_client SimpleActionClient : Client to collision free arm commands
        \param ranges { str : (float, float) } : Position ranges for each joint.
        \param recovery_client SimpleActionClient : Client to unsafe arm controller
        \param recovery_positions { str : float } : Recovery positions for each joint
        """
        self._arm_client = arm_client
        self._ranges = ranges
        self._recovery_client = recovery_client
        self._recovery_positions = recovery_positions

        self._fail_count = 0
        
        
    ##\return GoalStatus
    def command_goal(self, goal, duration = 15):
        self._arm_client.send_goal(goal)
        self._arm_client.wait_for_result(rospy.Duration.from_sec(duration))

        rv = self._arm_client.get_state()
        self._arm_client.cancel_goal()

        return rv

    ## Unit testing only
    ##\return GoalStatus
    def command_recovery(self):
        recovery_goal = _get_recovery_goal(self._recovery_positions)
        self._recovery_client.cancel_all_goals()
        self._recovery_client.send_goal(recovery_goal)
        rospy.loginfo('Sent recovery goal')
        self._recovery_client.wait_for_result(rospy.Duration.from_sec(10))

        rv = self._recovery_client.get_state()
        rospy.loginfo('Got recovery result: %d' % rv)

        self._recovery_client.cancel_goal()

        return rv

    def _get_random_goal(self):
        goal = JointTrajectoryGoal()
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration.from_sec(1)

        goal.trajectory.points.append(point)

        for joint, rng in self._ranges.iteritems():
            goal.trajectory.joint_names.append(joint)
                        
            goal.trajectory.points[0].positions.append(random.uniform(rng[0], rng[1]))

        return goal

    ## Send random commands to arm.
    def send_cmd(self):
        goal = self._get_random_goal()
        my_result = self.command_goal(goal)

        if my_result == GoalStatus.SUCCEEDED:
            self._fail_count = 0
        else:
            self._fail_count += 1

        if self._fail_count > FAILED_OUT:
            rospy.logwarn('Arm may be stuck. Sending recovery goal to free it')
            self._fail_count = 0
            self._arm_client.cancel_goal()
            self.command_recovery()

