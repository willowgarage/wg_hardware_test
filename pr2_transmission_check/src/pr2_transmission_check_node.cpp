/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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


#include "pr2_transmission_check/pr2_transmission_check_node.h"
#include <exception>
#include <limits>
#include "angles/angles.h"

using namespace pr2_transmission_check;
using namespace std;

// Diagnostic publisher
PR2TransmissionCheckNode::PR2TransmissionCheckNode() :
  pnh_("~"),
  use_sim_time_(false),
  disable_controller_warnings_(false),
  trans_status_(true)
{ 
  mech_st_sub_ = n_.subscribe("mechanism_statistics", 1000, &PR2TransmissionCheckNode::mechCallback, this);
  diag_pub_ = n_.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
  
  n_.param("/use_sim_time", use_sim_time_, false);

  pnh_.param("disable_controller_warnings", disable_controller_warnings_, false);

  TiXmlDocument xml;
  string robot_desc;
  if (!n_.getParam("robot_description", robot_desc))
  {
    ROS_ERROR("Unable to get parameters \"robot_description\" from server");
    throw PR2TransmissionCheckException("Unable to get parameters \"robot_description\" from server");
  }

  if (!xml.Parse(robot_desc.c_str()))
  {
    ROS_ERROR("Unable to parse XML for \"robot_description\" parameter");
    throw PR2TransmissionCheckException("Unable to parse \"robot_description\" XML");
  }

  TiXmlElement *robot = xml.FirstChildElement("robot");
  if (!robot)
  {
    ROS_ERROR("Unable to find \"robot\" element in URDF");
    throw PR2TransmissionCheckException("Unable to find element \"robot\" in \"robot_description\" XML");
  }

  // Check URDF init
  urdf::Model urdf_bot;
  if (!urdf_bot.initXml(robot))
  {
    ROS_ERROR("Unable to initialize URDF from robot_description");
    throw PR2TransmissionCheckException("Unable to initialize URDF from robot_description");
  }

  map<string, string> jnts_to_motors;
  loadTransmissions(robot, jnts_to_motors);

  loadTransCheckers(urdf_bot, jnts_to_motors);


  reset_srv_ = pnh_.advertiseService("reset_transmission_check", &PR2TransmissionCheckNode::reset_cb, this);

  trans_status_pub_ = pnh_.advertise<std_msgs::Bool>("transmission_status", 1, true);
}

void PR2TransmissionCheckNode::loadTransCheckers(urdf::Model &robot, map<string, string> &joints_to_actuators)
{
  map<string, boost::shared_ptr<urdf::Joint> >::iterator it;
  for (it = robot.joints_.begin(); it != robot.joints_.end(); ++it)
  {
    string &jnt_name = it->second->name;
    if (!joints_to_actuators.count(jnt_name))
    {
      ROS_DEBUG("Joint \"%s\" wasn't found in joints to actuators map.", jnt_name.c_str());
      continue;
    }
    
    if (!it->second->calibration)
    {
      ROS_DEBUG("Joint \"%s\" doesn't have calibration. Ignoring tranmission status.", jnt_name.c_str());
      continue;
    }

    string &actuator_name = joints_to_actuators[jnt_name];
    boost::shared_ptr<TransmissionListener> tl(new TransmissionListener());
    if (!tl->initUrdf(it->second, actuator_name))
    {
      ROS_ERROR("Unable to initialize transmission listener for joint \"%s\".", jnt_name.c_str());
      continue;
    }
    trans_listeners_.push_back(tl);
  }
}


void PR2TransmissionCheckNode::mechCallback(const pr2_mechanism_msgs::MechanismStatistics::ConstPtr& mechMsg)
{
  for (uint i = 0; i < trans_listeners_.size(); ++i)
    trans_listeners_[i]->update(mechMsg);
}

void PR2TransmissionCheckNode::loadTransmissions(TiXmlElement *robot, map<string, string> &joints_to_actuators)
{
  TiXmlElement *xit = NULL;
  for (xit = robot->FirstChildElement("transmission"); xit;
       xit = xit->NextSiblingElement("transmission"))
  {
    std::string type(xit->Attribute("type"));
    
    if (type.find("SimpleTransmission") != string::npos)
    {
      TiXmlElement *jel = xit->FirstChildElement("joint");
      if (!jel)
      {
        ROS_ERROR("SimpleTransmission did not specify joint element.");
        continue;
      }
      string joint_name(jel->Attribute("name"));


      TiXmlElement *ael = xit->FirstChildElement("actuator");
      if (!ael)
      {
        ROS_ERROR("SimpleTransmission did not specify actuator element.");
        continue;
      }
      string actuator_name(ael->Attribute("name"));

      if (joint_name.size() == 0 || actuator_name.size() == 0)
      {
        ROS_ERROR("SimpleTransmission did not give joint or actuator name");
        continue;
      }

      joints_to_actuators[joint_name] = actuator_name;
      ROS_DEBUG("Loaded \"%s\" : \"%s\" to map", joint_name.c_str(), actuator_name.c_str());
    }
    else if (type.find("WristTransmission") != string::npos)
    {
      TiXmlElement *ract = xit->FirstChildElement("rightActuator");
      if (!ract)
      {
        ROS_ERROR("WristTransmission did not specify rightActuator element.");
        continue;
      }
      string r_actuator(ract->Attribute("name"));

      TiXmlElement *lact = xit->FirstChildElement("leftActuator");
      if (!lact)
      {
        ROS_ERROR("WristTransmission did not specify leftActuator element.");
        continue;
      }
      string l_actuator(lact->Attribute("name"));

      // Joints
      TiXmlElement *flex_j = xit->FirstChildElement("flexJoint");
      if (!flex_j)
      {
        ROS_ERROR("WristTransmission did not specify flexJoint element.");
        continue;
      }
      string flex_joint(flex_j->Attribute("name"));

      TiXmlElement *roll_j = xit->FirstChildElement("rollJoint");
      if (!roll_j)
      {
        ROS_ERROR("WristTransmission did not specify rollJoint element.");
        continue;
      }
      string roll_joint(roll_j->Attribute("name"));

      // Add wrist transmission to map
      joints_to_actuators[flex_joint] = l_actuator;
      joints_to_actuators[roll_joint] = r_actuator;

      ROS_DEBUG("Loaded wrist transmission \"%s\" : \"%s\" into map", flex_joint.c_str(), l_actuator.c_str());
      ROS_DEBUG("Loaded wrist transmission \"%s\" : \"%s\" into map", roll_joint.c_str(), r_actuator.c_str());
    }
  }


}

void PR2TransmissionCheckNode::publishDiag()
{
  diagnostic_msgs::DiagnosticArray array;
  
  // Update with transmissions
  bool status = true;
  for (uint i = 0; i < trans_listeners_.size(); ++i)
  {
    array.status.push_back(*(trans_listeners_[i]->toDiagStat()));
    status = status && trans_listeners_[i]->checkOK();
  }

  array.header.stamp = ros::Time::now();
  diag_pub_.publish(array);

  // Halt motors if transmissions are broken
  if (!status && trans_status_)
  {
    ROS_ERROR("Halting motors, broken transmission. Check diagnostics");

    std_srvs::Empty::Request req;
    std_srvs::Empty::Response resp;
    ros::service::call("pr2_etherCAT/halt_motors", req, resp);
    
    trans_status_ = status;
  }

  // Publish transmission status
  std_msgs::Bool t_stat;
  t_stat.data = trans_status_;
  trans_status_pub_.publish(t_stat);

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_transmission_check");
  
  try
  {
    PR2TransmissionCheckNode ctrlJntPub;
    
    ros::Rate pub_rate(1.0);
    while (ctrlJntPub.ok())
    {
      pub_rate.sleep();
      ros::spinOnce();
      ctrlJntPub.publishDiag();
    }
  }
  catch (PR2TransmissionCheckException &e)
  {
    ROS_FATAL("PR2TransmissionCheck node caught TransmissionCheckException. Aborting. %s", e.what());
    ROS_BREAK();
  }
  catch (exception& e)
  {
    ROS_FATAL("PR2TransmissionCheck node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }
  
  exit(0);
  return 0;
}
