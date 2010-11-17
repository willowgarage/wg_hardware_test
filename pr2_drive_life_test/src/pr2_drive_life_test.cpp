/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// Author: Kevin Watts, Eric Berger

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

class PR2DriveLifeTest
{
private:
  std::string robot_frame_id_, map_frame_id_;

  geometry_msgs::PoseWithCovarianceStamped initial_pose_;

  ros::NodeHandle nh_;
  tf::TransformListener tf_;
  bool drive_halted_;
  ros::Time start_time_;

  double x_gain_, y_gain_, theta_gain_;
  double x_rng_, y_rng_, theta_rng_;
  double x_freq_, y_freq_, theta_freq_;

  ros::Publisher pose_pub_, cmd_pub_;
  ros::Subscriber motors_sub_;
  ros::ServiceServer halt_driving_, reset_driving_;
  

  bool haltDriveCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) { drive_halted_ = true; return true; }

  bool resetDriveCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) { drive_halted_ = false; sendInitialPose(); return true; }

  void sendInitialPose() 
  {
    pose_pub_.publish(initial_pose_);
  }

  void motorsCB(const std_msgs::Bool &msg)
  {
    if (msg.data) 
      drive_halted_ = true;
  }

  geometry_msgs::PoseStamped getTarget()
  {
    float dur = (ros::Time::now() - start_time_).toSec();

    geometry_msgs::PoseStamped rv;
    rv.pose.position.x = initial_pose_.pose.pose.position.x + sin(dur / 2.0 * M_PI * x_freq_) * x_rng_;
    rv.pose.position.y = initial_pose_.pose.pose.position.y + sin(dur / 2.0 * M_PI * y_freq_) * y_rng_;
    float theta = tf::getYaw(initial_pose_.pose.pose.orientation) + sin(dur / 2.0 * M_PI * theta_freq_) * theta_rng_;
    tf::Quaternion qt = tf::createQuaternionFromYaw(theta);
    tf::quaternionTFToMsg(qt, rv.pose.orientation);

    rv.header.frame_id = map_frame_id_;
    rv.header.stamp = ros::Time::now();

    return rv;
  }

  void pubPose(const geometry_msgs::PoseStamped &ps)
  {
    pose_pub_.publish(ps);
  }

  void commandBase(const geometry_msgs::PoseStamped &ps)
  {
    geometry_msgs::PoseStamped local_target;
    tf_.transformPose(robot_frame_id_, ps, local_target);

    geometry_msgs::Twist base_cmd;
    base_cmd.linear.y = y_gain_ * local_target.pose.position.y;
    // Check for big enough x to prevent wheel slip
    if (base_cmd.linear.y > 0.1)
      base_cmd.linear.x = x_gain_ * local_target.pose.position.x;

    // Don't command yaw if we're not translating
    if (abs(base_cmd.linear.x) + abs(base_cmd.linear.y) > 0.1)
      base_cmd.angular.z = tf::getYaw(ps.pose.orientation);

    if (!drive_halted_)
      cmd_pub_.publish(base_cmd);
  }

public:
  PR2DriveLifeTest() :
    drive_halted_(false),
    x_gain_(3.0),
    y_gain_(3.0),
    theta_gain_(3.0),
    x_rng_(0.0),
    y_rng_(0.6),
    theta_rng_(0.0),
    x_freq_(1.1),
    y_freq_(0.52),
    theta_freq_(1.1)
  { 

    initial_pose_.pose.pose.position.x = 2.207;
    initial_pose_.pose.pose.position.x = 5.253;

    tf::Quaternion qt = tf::createQuaternionFromYaw(3.071);
    tf::quaternionTFToMsg(qt, initial_pose_.pose.pose.orientation);
    // Same initial covariances from rviz
    initial_pose_.pose.covariance[0] = 0.25;
    initial_pose_.pose.covariance[7] = 0.25;
    initial_pose_.pose.covariance[21] = 0.068;
    
    // Publish pose, cmd_vel
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("target", 5, true);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5, true);
    
    // Sub to motors_halted
    motors_sub_ = nh_.subscribe("pr2_etherCAT/motors_halted", 10, &PR2DriveLifeTest::motorsCB, this);

    // Advertise services
    halt_driving_ = nh_.advertiseService("pr2_base/halt_drive", &PR2DriveLifeTest::haltDriveCB, this);
    reset_driving_ = nh_.advertiseService("pr2_base/reset_drive", &PR2DriveLifeTest::resetDriveCB, this);
  }
  




  ~PR2DriveLifeTest() { }

  void update()
  {
    geometry_msgs::PoseStamped target = getTarget();
    pubPose(target);
    commandBase(target);
  }

};

int main(int argv, char **argc)
{
  PR2DriveLifeTest pdlt;
  
  ros::Rate my_rate(0.002);
  while (ros::ok())
  {
    pdlt.update();
    ros::spinOnce();
  }

}
