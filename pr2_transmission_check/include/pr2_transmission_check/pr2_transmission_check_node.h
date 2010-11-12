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

///\author Kevin Watts
///\brief Checks transmissions for PR2 joints/transmissions

#ifndef _PR2_TRANSMISSION_CHECK_TRANS_CHECK_NODE_H_
#define _PR2_TRANSMISSION_CHECK_TRANS_CHECK_NODE_H_

#include <ros/ros.h>
#include <pr2_mechanism_msgs/MechanismStatistics.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <vector>
#include <float.h>
#include <boost/shared_ptr.hpp>
#include <boost/math/special_functions/fpclassify.hpp>
#include "pr2_mechanism_model/robot.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"
#include "pr2_transmission_check/transmission_check.h"
#include <exception>

namespace pr2_transmission_check
{

class PR2TransmissionCheckException: public std::runtime_error
{
public:
  PR2TransmissionCheckException(const std::string &errorDesc) : std::runtime_error(errorDesc) { ; };
};

/**
 * Publishes diagnostics data from pr2_mechanism_msgs/MechanismStatistics
 * for joints and controllers.
 */
class PR2TransmissionCheckNode
{
private:
  ros::NodeHandle n_, pnh_;
  ros::Subscriber mech_st_sub_;
  ros::Publisher diag_pub_;
  ros::Publisher trans_status_pub_;
  ros::ServiceServer reset_srv_;

  bool use_sim_time_;
  bool disable_controller_warnings_;

  std::vector<boost::shared_ptr<TransmissionListener> > trans_listeners_;
  bool trans_status_;

  void loadTransCheckers(urdf::Model &robot, std::map<std::string, std::string> &joints_to_actuators);

  void loadTransmissions(TiXmlElement *robot, std::map<std::string, std::string> &joints_to_actuators);
  
  void mechCallback(const pr2_mechanism_msgs::MechanismStatistics::ConstPtr& mechMsg);

  bool reset_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    for (uint i = 0; i < trans_listeners_.size(); ++i)
      trans_listeners_[i]->reset();

    trans_status_ = true;
    return true;
  }

public:
  PR2TransmissionCheckNode(); 

  ~PR2TransmissionCheckNode() { }
  

  
  void publishDiag(); /**< Publish diagnostics from transmissions **/

  bool ok() const { return n_.ok(); } 
};

}

#endif
