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


#ifndef _WGTEST_STATUS_INDICATOR_DELCOM_USB_LIGHT_H_
#define _WGTEST_STATUS_INDICATOR_DELCOM_USB_LIGHT_H_

#include <stdbool.h>
#include <hid.h>



/* Controller for Delcom USB light.
 *
 *
 */

namespace wgtest_status_indicator {

// Struct to send commands
class USBLightCommand
{
public:
  bool green, red, orange, off;

  USBLightCommand() :
    green(false),
    red(false),
    orange(false),
    off(false)
  {}
};

/* Controls a Delcom USB light, item #904003
 * http://www.delcom-products.com/productdetails.asp?productnum=904003
 *
 * Device is opened at startup, and if we detect the device is closed, we 
 * can force open it.
 *
 */
class DelcomUSBLight
{
private:
  HIDInterface *hid_dev_;

public:
  DelcomUSBLight();

  ~DelcomUSBLight();

  // Open is called before get/set commands. Can be called manually if needed
  bool openDevice(); 

  // Return true if we're open
  bool isOpen() const { return hid_dev_ != NULL; }
  
  // Return true if command sent successfully
  bool sendCommand(const USBLightCommand &cmd);  

  // Return false if unable to get status
  bool getStatus(USBLightCommand &status);
};



} // namespace

#endif //  _WGTEST_STATUS_INDICATOR_DELCOM_USB_LIGHT_H_
