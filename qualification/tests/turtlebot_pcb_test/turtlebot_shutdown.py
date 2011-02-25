#!/usr/bin/env python



import roslib
import wx
roslib.load_manifest('qualification')

import rospy
from pr2_self_test_msgs.srv import * 


rospy.wait_for_service('shutdown_done', 3)

app = wx.PySimpleApp()

done_proxy = rospy.ServiceProxy('shutdown_done', ScriptDone)
ret = wx.MessageBox("Power Down", "Turn off the Create by pressing the power button", wx.OK)
done = ScriptDoneRequest()
done.result = ScriptDoneRequest.RESULT_OK
done.failure_msg = ''

# Comment out for timeout check
done_proxy.call(done)
