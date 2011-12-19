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
#include <boost/foreach.hpp>
#include "pluginlib/class_list_macros.h"

namespace sine_controller
{

class SineController : public pr2_controller_interface::Controller
{
public:
  SineController();
  ~SineController();
  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  void starting();
  void update();

private:
  pr2_mechanism_model::RobotState *robot_;
  pr2_hardware_interface::Actuator *actuator_;
  pr2_mechanism_model::JointState *joint_state_; // Not currently used


  ros::Time initial_time_;
  double frequency_;
  double amplitude_;
  double offset_;
};

SineController::~SineController() 
{
}

SineController::SineController() : 
  robot_(NULL), actuator_(NULL), joint_state_(NULL), 
  frequency_(0.0), amplitude_(0.0), offset_(0.0)
{
  initial_time_ = ros::Time(0);
}

bool SineController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  if (!robot) 
    return false;
  robot_ = robot;
  
  std::string name;
  if (!n.getParam("joint_name", name)){
    ROS_ERROR("SineController: No 'joint_name' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!(joint_state_ = robot->getJointState(name)))
  {
    ROS_ERROR("SineController could not find joint named \"%s\"\n", name.c_str());
    return false;
  }
  
  /*
  std::string name;
  if (!n.getParam("actuator_name", name)){
    ROS_ERROR("SineController: No 'actuator_name' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  pr2_hardware_interface::HardwareInterface* hw = robot_->model_->hw_;
  actuator_ = hw->getActuator(name);
  if (actuator_ == NULL) {
    ROS_ERROR("SineController: Could not find actuator named : %s)", name.c_str());
    BOOST_FOREACH(const pr2_hardware_interface::ActuatorMap::value_type &v, hw->actuators_)
    {
      ROS_INFO("actuator : %s", v.first.c_str());
    }
    return false;
  }
  */

  if (!n.getParam("frequency", frequency_)){
    ROS_ERROR("SineController: No 'frequency' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("amplitude", amplitude_)){
    ROS_ERROR("SineController: No 'amplitude' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("offset", offset_)){
    ROS_ERROR("SineController: No 'offset' found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  initial_time_ = robot_->getTime();

  return true;
}

void SineController::starting()
{
  initial_time_ = robot_->getTime();
}

void SineController::update()
{
  double time = (robot_->getTime() - initial_time_).toSec();   
  double command = offset_ + amplitude_ * sin(time * frequency_ * 2.0 * M_PI);
  joint_state_->commanded_effort_ += command; 
  //actuator_->command_.effort_ = command;
}

PLUGINLIB_REGISTER_CLASS(SineController, sine_controller::SineController, pr2_controller_interface::Controller)


}//namespace sine_controller
