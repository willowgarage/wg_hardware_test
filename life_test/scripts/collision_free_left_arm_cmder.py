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
##\brief Sends goals to left arm to move it in collision free way

PKG = 'life_test'
import roslib; roslib.load_manifest(PKG)
import rospy
import actionlib
from life_test.commanders.arm_cmder import ArmCmder

from pr2_controllers_msgs.msg import JointTrajectoryAction
from arm_navigation_msgs.srv import SetPlanningSceneDiffRequest, SetPlanningSceneDiff

arm_ranges = {
    'l_shoulder_pan_joint': (-0.4, 2.0),
    'l_shoulder_lift_joint': (-0.4, 1.25),
    'l_upper_arm_roll_joint': (-0.3, 3.5),
    'l_elbow_flex_joint': (-2.0, -0.05) 
}

recovery_positions = {
    'l_shoulder_pan_joint': 0.8,
    'l_shoulder_lift_joint': 0.3,
    'l_upper_arm_roll_joint': 0.0,
    'l_elbow_flex_joint': -0.2
}

if __name__ == '__main__':
    rospy.init_node('arm_cmder_client')
    client = actionlib.SimpleActionClient('collision_free_arm_trajectory_action_left_arm', 
                                          JointTrajectoryAction)
    rospy.loginfo('Waiting for server for collision free arm commander')
    client.wait_for_server()

    rospy.loginfo('Waiting for environment server')

    rospy.wait_for_service('environment_server_left_arm/set_planning_scene_diff')

    set_planning_scene_diff_client = rospy.ServiceProxy('environment_server_left_arm/set_planning_scene_diff',
                                                        SetPlanningSceneDiff)

    # set up empty planning scene?
    planning_scene_diff_request = SetPlanningSceneDiffRequest()
    set_planning_scene_diff_client.call(planning_scene_diff_request)
  
    rospy.loginfo('Sending arm commands')

    my_rate = rospy.Rate(1.0)

    recovery_client = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action',
                                                JointTrajectoryAction)

    cmder = ArmCmder(client, arm_ranges, recovery_client, recovery_positions)

    while not rospy.is_shutdown():
        cmder.send_cmd()         
        my_rate.sleep()
