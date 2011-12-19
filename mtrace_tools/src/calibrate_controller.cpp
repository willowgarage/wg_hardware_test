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
#include <std_msgs/Empty.h>
#include <pr2_controller_interface/controller.h>
#include <boost/foreach.hpp>
#include "pluginlib/class_list_macros.h"

#define PKG mtrace_tools

namespace calibrate_controller
{

class CalibrateController : public pr2_controller_interface::Controller
{
public:
  CalibrateController();
  ~CalibrateController();
  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
  void starting();
  void update();
  void triggerCB(std_msgs::EmptyConstPtr const &e) 
  { 
    ROS_ERROR("Trigger");
    trigger_=true; 
  }

private:
  pr2_mechanism_model::RobotState *robot_;
  pr2_hardware_interface::Actuator *actuator_;
  pr2_mechanism_model::JointState *joint_state_; // Not currently used
  ros::Subscriber subscriber_;
  ros::NodeHandle node_;
  bool trigger_;
};

CalibrateController::~CalibrateController() 
{
}

CalibrateController::CalibrateController() : 
  robot_(NULL), actuator_(NULL), joint_state_(NULL), trigger_(false)
{
}

bool CalibrateController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &node)
{
  if (!robot) 
    return false;
  robot_ = robot;
  
  std::string name;
  if (!node.getParam("actuator_name", name)){
    ROS_ERROR("CalibrateController: No 'actuator_name' found on parameter namespace: %s)",
              node.getNamespace().c_str());
    return false;
  }
  pr2_hardware_interface::HardwareInterface* hw = robot_->model_->hw_;
  actuator_ = hw->getActuator(name);
  if (actuator_ == NULL) {
    ROS_ERROR("CalibrateController: Could not find actuator named : %s)", name.c_str());
    BOOST_FOREACH(const pr2_hardware_interface::ActuatorMap::value_type &v, hw->actuators_)
    {
      ROS_INFO("actuator : %s", v.first.c_str());
    }
    return false;
  }

  node_ = node;
  subscriber_ = node.subscribe("trigger", 1u, &CalibrateController::triggerCB, this);

  return true;
}

void CalibrateController::starting()
{
}

void CalibrateController::update()
{
  if (!actuator_)
    return;

  if (trigger_) 
  {
    trigger_ = false;
    ROS_ERROR("Trigger zero = %f, position = %f", actuator_->state_.zero_offset_, actuator_->state_.position_);
    actuator_->state_.zero_offset_ = actuator_->state_.position_;
  }
}

//PLUGINLIB_DECLARE_CLASS(PKG, CalibrateController, calibrate_controller::CalibrateController, pr2_controller_interface::Controller)
PLUGINLIB_REGISTER_CLASS(CalibrateController, calibrate_controller::CalibrateController, pr2_controller_interface::Controller)


}//namespace calibrate_controller
