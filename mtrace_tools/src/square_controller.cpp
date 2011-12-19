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

#include "pluginlib/class_list_macros.h"

namespace square_controller
{

class SquareController : public pr2_controller_interface::Controller
{
public:
  SquareController();
  ~SquareController();
  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  void starting();
  void update();

private:
  pr2_mechanism_model::JointState *joint_state_;
  pr2_mechanism_model::RobotState *robot_;

  ros::Time initial_time_;
  double frequency_;
  double amplitude_;
  double offset_;
};

SquareController::~SquareController() 
{
}

SquareController::SquareController() : joint_state_(NULL), frequency_(0.0), amplitude_(0.0), offset_(0.0)
{
  initial_time_ = ros::Time(0);
}

bool SquareController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  if (!robot) 
    return false;
  robot_ = robot;


  std::string name;
  if (!n.getParam("joint_name", name)){
    ROS_ERROR("SquareController: No 'joint_name' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!(joint_state_ = robot->getJointState(name)))
  {
    ROS_ERROR("SquareController could not find joint named \"%s\"\n", name.c_str());
    return false;
  }

  if (!n.getParam("frequency", frequency_)){
    ROS_ERROR("SquareController: No 'frequency' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("amplitude", amplitude_)){
    ROS_ERROR("SquareController: No 'amplitude' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("offset", offset_)){
    ROS_ERROR("SquareController: No 'offset' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  initial_time_ = robot_->getTime();

  return true;
}

void SquareController::starting()
{
  initial_time_ = robot_->getTime();
}

void SquareController::update()
{
  double time = (robot_->getTime() - initial_time_).toSec();   
  double square = (fmod(time*frequency_, 1.0) > 0.5) ? 1.0 : -1.0;
  double command = offset_ + amplitude_ * square;
  joint_state_->commanded_effort_ += command; 
}

PLUGINLIB_REGISTER_CLASS(SquareController, square_controller::SquareController, pr2_controller_interface::Controller)


}//namespace square_contoller
