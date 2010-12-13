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

#include <string.h>
#include <stdio.h>
#include <getopt.h>
#include <stdbool.h>
#include <string>

#include  <wgtest_status_indicator/delcom_usb_light.h>

using namespace wgtest_status_indicator;

std::string boolToString(bool val)
{
  return val ? "True" : "False";
}

int main (int argc, char **argv)
{
    // First, grab the user's options.
    static int orange_flag;
    static int get_flag;
    static int green_flag;
    static int help_flag;
    static int off_flag;
    static int red_flag;

    int c;
    while (1) 
    {
      static struct option long_options[] = {
        // These options set a flag.
        {"orange",  no_argument,  &orange_flag,  1},
        {"get",   no_argument,  &get_flag,   1},
        {"green", no_argument,  &green_flag, 1},
        {"help",  no_argument,  &help_flag,  1},
        {"off",   no_argument,  &off_flag,   1},
        {"red",   no_argument,  &red_flag,   1},
      };
      
      int option_index = 0;
      c = getopt_long(argc, argv, "", long_options, &option_index);
      
      if (c == -1)
        break;
      
      switch (c) 
      {
      case 0:
        if (long_options[option_index].flag != 0)
          break;
      }
    }
    
    // Print usage if the user didn't supply a valid option or asked for help.
    if ((! orange_flag && ! red_flag && ! green_flag && ! off_flag && ! get_flag)
        || help_flag) 
    {
      printf(
             "Usage: %s [ --orange | --red | --green | --off | --get ]\n",
             argv[0]
             );
      
      exit(1);
    }
    
    DelcomUSBLight *light = new DelcomUSBLight;
    if (!light->isOpen())
    {
      fprintf(stderr, "Unable to open USB light!");
      return 1;
    }
    
    // Make command
    USBLightCommand cmd;

    if (get_flag)
    {
      // Get status
      if (!light->getStatus(cmd))
        return 1;

      if (cmd.off)
        fprintf(stdout, "Light status: OFF\n");
      else
      {
        fprintf(stdout, "Green: %s. Orange: %s. Red: %s.\n", 
                boolToString(cmd.green).c_str(),
                boolToString(cmd.orange).c_str(), 
                boolToString(cmd.red).c_str());
      }
    }
    else
    {
      // Turn light on/off
      cmd.green = green_flag;
      cmd.orange = orange_flag;
      cmd.off = off_flag;
      cmd.red = red_flag;
      
      if (!light->sendCommand(cmd))
        return 1;
    }

    return 0;
}
