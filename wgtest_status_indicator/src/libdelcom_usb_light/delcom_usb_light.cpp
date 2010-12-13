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


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <hid.h>

#include <ros/console.h>

#include <wgtest_status_indicator/delcom_usb_light.h>

using namespace wgtest_status_indicator;

bool DelcomUSBLight::openDevice()
{
    // These are for a delcom gen 2 usb light. YMMV.
  static const unsigned short vendor_id  = 0x0fc5;
  static const unsigned short product_id = 0xb080;

  // Clear the existing device, if any
  if (hid_dev_)
  {
    delete hid_dev_;
    hid_dev_ = NULL;
  }  

  HIDInterfaceMatcher matcher = { vendor_id, product_id, NULL, NULL, 0 };
  
  int iface_num = 0;
  hid_return ret;

  HIDInterface *iface;

  ret = hid_init();
  if (ret != HID_RET_SUCCESS) 
  {
    ROS_ERROR("hid_init failed with return code %x", ret);
    return false;
  }
  
  iface = hid_new_HIDInterface();
  if (iface == 0) 
  {
    ROS_ERROR("hid_new_HIDInterface() failed, out of memory?");
    return false;
  }
  
  ret = hid_force_open(iface, iface_num, &matcher, 3);
  if (ret == 12)
  {
    ROS_ERROR("hid_force_open failed with error %x. This may be a permissions issue. Check device permissions", ret);
    return false;
  }
  else if (ret != HID_RET_SUCCESS) 
  {
    fprintf(stderr, "hid_force_open failed with return code %x", ret);
    return false;
  }

  // Set our device up
  hid_dev_ = iface;
  return true;
}

bool DelcomUSBLight::sendCommand(const USBLightCommand &cmd)
{
  if (!isOpen() && !openDevice())
  {
    ROS_ERROR("Unable to open device to send command");
    return false;
  }

  // Path to the LEDs
  int path[2];
  path[0] = 0xff000000;
  path[1] = 0x00000000;
  
  // Set up the packet
  uint8_t buf[8];  
  memset(buf, 0, sizeof(buf));

  buf[0] = 101;
  buf[1] = 12;
  
  // buf[2] is the LSB, buf[3] is the MSB
  if (cmd.green)
  {
    buf[2] = 0x01 << 0;
    buf[3] = 0xFF;
  }
  if (cmd.red)
  {
    buf[2] = 0x01 << 1;
    buf[3] = 0xFF;
  }
  if (cmd.orange)
  {
    buf[2] = 0x01 << 2;
    buf[3] = 0xFF;
  }
  if (cmd.off)
  {
    buf[3] = 0xFF;
  }
  
  hid_return ret = hid_set_feature_report(hid_dev_, path, 1, (char*)buf, 8);
  if (ret != HID_RET_SUCCESS) 
  {
    ROS_ERROR("hid_set_feature_report failed(). Returned %d", ret);
    return false;
  }

  return true;
}


bool DelcomUSBLight::getStatus(USBLightCommand &status)
{
  if (!isOpen() && !openDevice())
  {
    ROS_ERROR("Unable to open device to get status");
    return false;
  }

  // Path to the LEDs
  int path[2];
  path[0] = 0xff000000;
  path[1] = 0x00000000;
  
  // Set up the packet
  uint8_t buf[8];  
  memset(buf, 0, sizeof(buf));

  // We are reading the feature to find out the current status.
  buf[0] = 100;

  hid_return ret = hid_get_feature_report(hid_dev_, path, 1, (char*)buf, 8);
  if (ret != HID_RET_SUCCESS) 
  {
    ROS_ERROR("hid_get_feature_report failed(). Returned: %x", ret);
    return false;
  }

  status.green  = buf[2] == 0xFE;
  status.red    = buf[2] == 0xFD;
  status.orange = buf[2] == 0xFB;
  
  if (!status.green && !status.red && !status.orange)
    status.off = true;

  return true;
}

DelcomUSBLight::DelcomUSBLight() :
  hid_dev_(NULL)
{
  openDevice();
}

DelcomUSBLight::~DelcomUSBLight()
{
  if (hid_dev_)
    delete hid_dev_;  
}
