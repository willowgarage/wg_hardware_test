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

/* Simple utility to control delcom USB light from command line
 *
 *
 */

#include  <wgtest_status_indicator/delcom_usb_light.h>

#include <ros/ros.h>
#include <string>

#include <pr2_self_test_msgs/TestInfoArray.h>

using namespace wgtest_status_indicator;

DelcomUSBLight *g_light;

ros::Time *g_last_callback;

void callback(const pr2_self_test_msgs::TestInfoArray &msg)
{
  int status = 0;
  for (unsigned int i = 0; i < msg.data.size(); ++i)
  {
    // Ignore tests that aren't launched
    if (msg.data[i].test_status > 3)
      continue; 

    status = std::max(status, (int) msg.data[i].test_status);
  }
  
  // Make command
  USBLightCommand cmd;
  if (status == 0)
    cmd.green = true;
  else if (status == 1)
    cmd.orange = true;
  else
    cmd.red = true;

  g_light->sendCommand(cmd);

  if (g_last_callback)
    delete g_last_callback;
  g_last_callback = new ros::Time(ros::Time::now());
}

int main(int argc, char **argv)
{
  g_last_callback = new ros::Time(0);

  ros::init(argc, argv, "wgtest_status_indicator");

  ros::NodeHandle nh;

  ros::Subscriber stat_sub = nh.subscribe("test_info", 5, &callback);

  g_light = new DelcomUSBLight();

  ros::Rate my_rate(1.0);

  USBLightCommand off_cmd;
  off_cmd.off = true;

  while (ros::ok())
  {
    ros::spinOnce();
    my_rate.sleep();

    if (ros::Time::now() - *g_last_callback > ros::Duration(30.0))
      g_light->sendCommand(off_cmd);
  }

  delete g_light;
}
