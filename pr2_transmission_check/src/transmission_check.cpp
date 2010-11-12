/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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


#include "pr2_transmission_check/transmission_check.h"
#include <limits>
#include "angles/angles.h"

#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>

using namespace pr2_transmission_check;
using namespace std;

/*** TODO
 * Set deadband properly for each joint
 * Figure out max/min
 * Track state with accumulators
 * Only halt motors if we've had >3 consecutive hits, etc
 * Add tests for warnings
 * Publish status topic (bool) at 1 Hz
 * 
 *****/

#define GRACE_HITS 5

// Transmission Listener
TransmissionListener::TransmissionListener() :
  joint_name_(""),
  actuator_name_(""),
  deadband_(0.1),
  has_wrap_(false),
  has_up_(false),
  has_down_(false),
  status_(true),
  last_trans_status_(true),
  error_cnt_(0),
  num_errors_(0),
  num_hits_(0),
  num_errors_since_reset_(0),
  rx_cnt_(0),
  last_rising_(0.0),
  last_falling_(0.0),
  last_bad_reading_(0.0),
  last_position_(0.0),
  is_calibrated_(false),
  has_updated_(false)
{ }

bool TransmissionListener::initUrdf(const boost::shared_ptr<urdf::Joint> jnt, const string &actuator_name)
{
  joint_name_ = jnt->name;
  actuator_name_ = actuator_name;
  
  if (!jnt->calibration)
  {
    ROS_DEBUG("Joint \"%s\" does not support calibration.", joint_name_.c_str());
    return false;
  }

  has_up_ = (bool) jnt->calibration->rising;
  has_down_ = (bool) jnt->calibration->falling;

  if (has_up_)
    up_ref_ = *(jnt->calibration->rising);

  if (has_down_)
    down_ref_ = *(jnt->calibration->falling);
  
  if (jnt->type == urdf::Joint::CONTINUOUS)
  {
    has_wrap_ = true;

    if (has_up_ && !has_down_)
      down_ref_ = up_ref_ + M_PI;
    if (!has_up_ && has_down_)
      up_ref_ = down_ref_ + M_PI;

    up_ref_ = angles::normalize_angle(up_ref_);
    down_ref_ = angles::normalize_angle(down_ref_);

    has_up_ = true;
    has_down_ = true;
  }

  return true;
}

// Look for joint, actuator
// Check bounds
bool TransmissionListener::update(const pr2_mechanism_msgs::MechanismStatistics::ConstPtr& mechMsg)
{
  const pr2_mechanism_msgs::JointStatistics *js = NULL;
  const pr2_mechanism_msgs::ActuatorStatistics *as = NULL;

  for (uint i = 0; i < mechMsg->joint_statistics.size(); ++i)
  {
    if (mechMsg->joint_statistics[i].name == joint_name_)
      js = &mechMsg->joint_statistics[i];
  }

  if (!js)
  {
    ROS_ERROR_ONCE("Unable to find joint state for joint \"%s\".", joint_name_.c_str());
    return false;
  }

  if (!js->is_calibrated)
    return true; // Ignore until we're calibrated

  // Same for actuators
  for (uint i = 0; i < mechMsg->actuator_statistics.size(); ++i)
  {
    if (mechMsg->actuator_statistics[i].name == actuator_name_)
      as = &mechMsg->actuator_statistics[i];
  }

  if (!as)
  {
    ROS_ERROR_ONCE("Unable to find actuator state for actuator \"%s\".", actuator_name_.c_str());
    return false;
  }

  rx_cnt_++;
  last_rising_ = as->last_calibration_rising_edge;
  last_falling_ = as->last_calibration_falling_edge;
  last_cal_reading_ = as->calibration_reading;
  last_position_ = js->position;
  is_calibrated_ = js->is_calibrated;
  position_obs_(last_position_);

  /* Don't check bounds because we have no max/min
  if (!checkBounds(js))
  {
    last_trans_status_ = false;
    status_ = false;
    return false;
  }
  */

  bool rv = checkFlag(js, as);

  last_trans_status_ = rv;
  if (!rv)
  {
    num_hits_++;
    last_bad_reading_ = last_position_;
    error_cnt_++;
  }
  else
    error_cnt_ = 0;
  
  if (error_cnt_ > GRACE_HITS)
  {
    num_errors_++;
    num_errors_since_reset_++;
    status_ = false;
  }

  has_updated_ = true;
  return true;
}

bool TransmissionListener::checkBounds(const pr2_mechanism_msgs::JointStatistics *js) const
{
  ROS_ASSERT_MSG(js->name == joint_name_, "Joint name didn't match!");

  // We should only see calibrated joints
  if (!js->is_calibrated)
  {
    ROS_ERROR_ONCE("Joint \"%s\" isn't calibrated. Unable to check bounds for joint.", joint_name_.c_str());
    return false; 
  }

  return js->position < max_ && js->position > min_;
}

bool TransmissionListener::checkFlag(const pr2_mechanism_msgs::JointStatistics *js,
                                     const pr2_mechanism_msgs::ActuatorStatistics *as) const
{
  ROS_ASSERT_MSG(js->name == joint_name_, "Joint name didn't match!");
  ROS_ASSERT_MSG(as->name == actuator_name_, "Actuator name didn't match!");

  float jnt_position = js->position;

  if (!has_up_ && !has_down_)
  {
    ROS_ERROR("Neither up or down reference specified for joint \"%s\". Unable to check transmission.",
              joint_name_.c_str());
    return false;
  }

  // Check if we need to wrap position
  if (has_wrap_)
    jnt_position = angles::normalize_angle(jnt_position);

  // Check deadband for up reference
  if (has_up_)
  {
    if (abs(jnt_position - up_ref_) < deadband_)
      return true; // Too close to deadband
  }
  if (has_down_)
  {
    if (abs(jnt_position - down_ref_) < deadband_)
      return true; // Too close to deadband
  }

  // Check that we're OK on other side of normalized angle
  if (has_up_ && has_down_ && has_wrap_)
  {
    if (abs((  2 * M_PI - jnt_position) - up_ref_)   < deadband_)
      return true;
    if (abs((- 2 * M_PI + jnt_position) - up_ref_)   < deadband_)
      return true;
    if (abs((  2 * M_PI - jnt_position) - down_ref_) < deadband_)
      return true;
    if (abs((- 2 * M_PI + jnt_position) - down_ref_) < deadband_)
      return true;
  }
  
  if (has_up_ && has_down_)
  {
    if (up_ref_ > down_ref_)
    {
      if (jnt_position < down_ref_ && as->calibration_reading)
        return true;
      else if (jnt_position > down_ref_ && jnt_position < up_ref_ && !as->calibration_reading)
        return true;
      else if (jnt_position > up_ref_ && as->calibration_reading)
        return true;
      else 
      {
        ROS_WARN_THROTTLE(1, "Broken transmission reading for \"%s\". Position: %f (wrapped), cal reading: %d. Up ref: %f, Down ref: %f", joint_name_.c_str(), jnt_position, as->calibration_reading, up_ref_, down_ref_);
        return false;
      }
    }
    else // down_ref_ > up_ref_
    {
      if (jnt_position < up_ref_ && !as->calibration_reading)
        return true;
      else if (jnt_position > up_ref_ && jnt_position < down_ref_ && as->calibration_reading)
        return true;
      else if (jnt_position > down_ref_ && !as->calibration_reading)
        return true;
      else 
      {
        ROS_WARN_THROTTLE(1, "Broken transmission reading for \"%s\". Position: %f (wrapped), cal reading: %d. Up ref: %f, Down ref: %f", joint_name_.c_str(), jnt_position, as->calibration_reading, up_ref_, down_ref_);
        return false;
      }
    }
  }
  else if (has_up_)
  {
    if (jnt_position > up_ref_ && as->calibration_reading)
      return true;
    else if (jnt_position < up_ref_ && !as->calibration_reading)
      return true;
    else
    {
      ROS_WARN_THROTTLE(1, "Broken transmission reading for \"%s\". Position: %f (wrapped), cal reading: %d. Up ref: %f, Down ref: %f", joint_name_.c_str(), jnt_position, as->calibration_reading, up_ref_, down_ref_);
        return false;
    }
  }

  else // has_down_
  {
    if (jnt_position > down_ref_ && !as->calibration_reading)
      return true;
    else if (jnt_position < down_ref_ && as->calibration_reading)
      return true;
    else
    {
      ROS_WARN_THROTTLE(1, "Broken transmission reading for \"%s\". Position: %f (wrapped), cal reading: %d. Up ref: %f, Down ref: %f", joint_name_.c_str(), jnt_position, as->calibration_reading, up_ref_, down_ref_);
        return false;
    }
  }
    
  ROS_ASSERT_MSG(false, "No case handled for transmission checker!");
  return false;
}

boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> TransmissionListener::toDiagStat() const
{
  boost::shared_ptr<diagnostic_updater::DiagnosticStatusWrapper> stat(new diagnostic_updater::DiagnosticStatusWrapper);
  
  stat->name = "Transmission (" + joint_name_ + ")";
      
  if (!has_updated_)
    stat->mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "No Updates");

  if (!status_)
    stat->mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Broken Transmission");

  stat->add("Transmission Status", status_ ? "OK" : "Broken");
  stat->add("Current Reading", last_trans_status_ ? "OK" : "Broken");
  
  stat->add("Joint", joint_name_);
  stat->add("Actuator", actuator_name_);
  if (has_up_)
    stat->add("Up Position", up_ref_);
  else
    stat->add("Up Position", "N/A");

  if (has_down_)
    stat->add("Down Position", down_ref_);
  else
    stat->add("Down Position", "N/A");

  stat->add("Wrapped", has_wrap_ ? "True" : "False");
  //stat->add("Max Limit", max_);
  //stat->add("Min Limit", min_);
 
  // Todo: Start tracking state....
  stat->add("Mech State RX Count", rx_cnt_);
  stat->add("Is Calibrated", is_calibrated_ ? "True" : "False");
  stat->add("Calibration Reading", last_cal_reading_);
  stat->add("Joint Position", last_position_);
  stat->add("Total Errors", num_errors_);
  stat->add("Errors Since Reset", num_errors_since_reset_);
  stat->add("Total Bad Readings", num_hits_);
  float max_obs_val = boost::accumulators::max( position_obs_ );
  float min_obs_val = boost::accumulators::min( position_obs_ );
  stat->add("Max Obs. Position", max_obs_val );
  stat->add("Min Obs. Position", min_obs_val );
  
  stat->add("Last Rising Edge", last_rising_);
  stat->add("Last Falling Edge", last_falling_);
  stat->add("Last Bad Reading", last_bad_reading_);
  

  return stat;
}

