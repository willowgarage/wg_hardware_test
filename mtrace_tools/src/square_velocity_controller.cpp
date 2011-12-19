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

#include <ros/ros.h>
#include <math.h>
#include <pr2_controller_interface/controller.h>
#include <control_toolbox/pid.h>

#include "pluginlib/class_list_macros.h"

namespace square_velocity_controller
{

class SquareVelocityController : public pr2_controller_interface::Controller
{
public:
  SquareVelocityController();
  ~SquareVelocityController();
  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  void starting();
  void update();

private:
  pr2_mechanism_model::JointState *joint_state_;
  pr2_mechanism_model::RobotState *robot_;
  control_toolbox::Pid pid_;

  ros::Time initial_time_;
  ros::Time period_start_time_;
  ros::Time last_update_time_;
  double target_position_;
  double period_;
  double amplitude_;
  double velocity_;
};

SquareVelocityController::~SquareVelocityController() 
{
}

SquareVelocityController::SquareVelocityController() : 
  joint_state_(NULL), 
  target_position_(0.0),
  period_(1.0), 
  amplitude_(0.0), 
  velocity_(0.0)
{
  initial_time_ = ros::Time(0);
  period_start_time_ = ros::Time(0);
  last_update_time_ = ros::Time(0);
}

bool SquareVelocityController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  if (!robot) 
    return false;
  robot_ = robot;

  std::string name;
  if (!n.getParam("joint_name", name)){
    ROS_ERROR("SquareVelocityController: No 'joint_name' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!(joint_state_ = robot->getJointState(name)))
  {
    ROS_ERROR("SquareVelocityController could not find joint named \"%s\"\n", name.c_str());
    return false;
  }

  double frequency;
  if (!n.getParam("frequency", frequency)){
    ROS_ERROR("SquareVelocityController: No 'frequency' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (frequency <= 1.0 || frequency > 500.0) 
  {
    ROS_ERROR("SquareVelocityController: Invalid frequency %f:)", frequency);
    return false;
  }
  period_ = 1.0/frequency;

  if (!n.getParam("amplitude", amplitude_)){
    ROS_ERROR("SquareVelocityController: No 'amplitude' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("velocity", velocity_)){
    ROS_ERROR("SquareVelocityController: No 'offset' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }  

  double p,i,d;
  if (!n.getParam("p", p)){
    ROS_ERROR("SquareVelocityController: No 'p' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }  
  if (!n.getParam("i", i)){
    ROS_ERROR("SquareVelocityController: No 'i' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }  
  if (!n.getParam("d", d)){
    ROS_ERROR("SquareVelocityController: No 'd' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  } 
  pid_.initPid(p,i,d, 1.0,-1.0);

  starting();
  return true;
}

void SquareVelocityController::starting()
{
  ros::Time time = robot_->getTime();
  initial_time_      = time;
  period_start_time_ = time;
  last_update_time_  = time;
  target_position_   = joint_state_->position_;
}

void SquareVelocityController::update()
{
  ros::Time current_time = robot_->getTime();
  double period_time = (current_time - period_start_time_).toSec();
  if (period_time > period_) {
    period_start_time_ += ros::Duration(period_);
    ros::Duration update_period = current_time - last_update_time_;
    target_position_ += velocity_ * update_period.toSec();
    double error = target_position_ - joint_state_->position_;
    pid_.updatePid(error, update_period); 
    last_update_time_ = current_time;
  }
  double duty = pid_.getCurrentCmd(); 
  double square = (period_time > (duty+1.0)*period_*0.5) ? 1.0 : -1.0;
  double command = amplitude_ * square;
  joint_state_->commanded_effort_ += command; 
}

PLUGINLIB_REGISTER_CLASS(SquareVelocityController, square_velocity_controller::SquareVelocityController, pr2_controller_interface::Controller)


}//namespace square_contoller
