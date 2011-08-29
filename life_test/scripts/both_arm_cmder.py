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
import copy
import math
from time import sleep

from pr2_controllers_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from arm_navigation_msgs.msg import Shape, CollisionObject, CollisionObjectOperation, AttachedCollisionObject, PlanningScene
from arm_navigation_msgs.srv import SetPlanningSceneDiffRequest, SetPlanningSceneDiff
from geometry_msgs.msg import Pose
from actionlib_msgs.msg import GoalStatus

from life_test.commanders.arm_cmder import ArmCmder

COLLISION_TOPIC = "collision_object"

# Shoulder pan joints are limited in position to prevent contact
ranges = {
    'r_shoulder_pan_joint': (-2.0, -0.4),
    'r_shoulder_lift_joint': (-0.3, 1.25),
    'r_upper_arm_roll_joint': (-3.1, 0),
    'r_elbow_flex_joint': (-2.0, -0.2),
    'r_forearm_roll_joint': (-3.14, 3.14),
    'r_wrist_flex_joint': (-1.8, -0.2),
    'r_wrist_roll_joint': (-3.14, 3.14),
    'l_shoulder_pan_joint': (0.4, 2.0),
    'l_shoulder_lift_joint': (-0.3, 1.25),
    'l_upper_arm_roll_joint': (0, 3.1),
    'l_elbow_flex_joint': (-2.0, -0.2),
    'l_forearm_roll_joint': (-3.14, 3.14),
    'l_wrist_flex_joint': (-1.8, -0.2),
    'l_wrist_roll_joint': (-3.14, 3.14)
}

recovery_positions = {
    'r_shoulder_pan_joint': - math.pi / 4,
    'r_shoulder_lift_joint': 0.0,
    'r_upper_arm_roll_joint': 0.0,
    'r_elbow_flex_joint': -.25,
    'r_forearm_roll_joint': 0.0,
    'r_wrist_flex_joint': -.25,
    'r_wrist_roll_joint': 0.0,
    'l_shoulder_pan_joint': math.pi / 4,
    'l_shoulder_lift_joint': 0.0,
    'l_upper_arm_roll_joint': 0.0,
    'l_elbow_flex_joint': -.25,
    'l_forearm_roll_joint': 0.0,
    'l_wrist_flex_joint': -.25,
    'l_wrist_roll_joint': 0.0
}

def get_virtual_gloves():
    r_glove = AttachedCollisionObject()
    r_glove.object.header.stamp = rospy.get_rostime()
    r_glove.object.header.frame_id = '/r_gripper_palm_link'
    r_glove.link_name = 'r_gripper_palm_link'

    r_glove.object.id = 'r_gripper_glove'
    r_glove.object.operation.operation = CollisionObjectOperation.ADD
    
    glove_shape = Shape()
    glove_shape.type = Shape.BOX
    glove_shape.dimensions = [ 0.25, 0.18, 0.1 ]
    glove_pose = Pose()
    glove_pose.orientation.w = 1
    glove_pose.position.x = 0.1
    
    # Pose will be zero

    r_glove.touch_links = ['r_end_effector',
                           'r_wrist_roll_link',
                           'r_wrist_flex_link',
                           'r_forearm_link']

    r_glove.object.shapes.append(glove_shape)
    r_glove.object.poses.append(glove_pose)

    l_glove = copy.deepcopy(r_glove)
    l_glove.object.id = 'l_gripper_glove'
    l_glove.object.header.frame_id = '/l_gripper_palm_link'
    l_glove.link_name = 'l_gripper_palm_link'
    l_glove.touch_links = ['l_end_effector',
                           'l_wrist_roll_link',
                           'l_wrist_flex_link',
                           'l_forearm_link']

    return r_glove, l_glove
    

##\brief Sets up a virtual table in front of the robot
def get_virtual_table(height = 0.42):
    table_msg = CollisionObject()

    table_msg.id = "table"
    table_msg.operation.operation = CollisionObjectOperation.ADD    

    table_msg.header.stamp = rospy.get_rostime()
    table_msg.header.frame_id = "base_footprint"
    
    side_box = Shape()
    side_box.type = Shape.BOX
    side_box.dimensions = [ 3.0, 1.0, height ]

    front_box = Shape()
    front_box.type = Shape.BOX
    front_box.dimensions = [ 1.0, 3.0, height ]

    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = height / 2
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1

    l_side_pose = copy.deepcopy(pose)
    l_side_pose.position.y = 0.85

    r_side_pose = copy.deepcopy(pose)
    r_side_pose.position.y = -0.85

    front_pose = copy.deepcopy(pose)
    front_pose.position.x = 0.85

    table_msg.shapes = [ side_box, side_box, front_box ]
    table_msg.poses = [ l_side_pose, r_side_pose, front_pose ]

    return table_msg

if __name__ == '__main__':
    rospy.init_node('arm_cmder_client')
    client = actionlib.SimpleActionClient('collision_free_arm_trajectory_action_arms', 
                                          JointTrajectoryAction)
    rospy.loginfo('Waiting for server for right collision free arm commander')
    client.wait_for_server()

    rospy.loginfo('Right, left arm commanders ready')

    rospy.loginfo('Waiting for environment server')

    rospy.wait_for_service('environment_server/set_planning_scene_diff')

    set_planning_scene_diff_client = rospy.ServiceProxy('environment_server/set_planning_scene_diff',
                                                        SetPlanningSceneDiff)

    planning_scene_diff_request = SetPlanningSceneDiffRequest()
    planning_scene_diff_request.planning_scene_diff.collision_objects.append(get_virtual_table())
    r_glove, l_glove = get_virtual_gloves()
    #planning_scene_diff_request.planning_scene_diff.attached_collision_objects.append(r_glove)
    #planning_scene_diff_request.planning_scene_diff.attached_collision_objects.append(l_glove)

    set_planning_scene_diff_client.call(planning_scene_diff_request)

    # Recovery trajectory client
    recovery_client = actionlib.SimpleActionClient('both_arms_controller/joint_trajectory_action',
                                                JointTrajectoryAction)
    
    cmder = ArmCmder(client, ranges, recovery_client, recovery_positions)

    while not rospy.is_shutdown():
        rospy.logdebug('Sending goal to arms.')
        cmder.send_cmd()

        #sleep(2.0)

