#! /usr/bin/env python

import roslib; roslib.load_manifest('life_test')
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import sys


class LastMessage():
    def __init__(self, topic, msg_type):
        self.msg = None
        rospy.Subscriber(topic, msg_type, self.callback)

    def last(self):
        return self.msg

    def callback(self, msg):
        self.msg = msg

start_pos = 0
end_pos = 0

fwd = -0.008
bck = 0.005
stop = 0

def calibrate():
    calibration = False    
    controller_name = rospy.myargv()[1]
    control_topic = '%s/command' % controller_name
    pub = rospy.Publisher(control_topic, Float64) 
    rospy.init_node('calibration')
    while calibration == False:
        
        last_state = LastMessage('joint_states', JointState)
        msg = last_state.last() 

        rospy.sleep(0.4)
        pub.publish(Float64(-.002))
        rospy.sleep(8.5)
        pub.publish(Float64(stop))
        
        msg = last_state.last()
        end_pos = msg.position[0]
        print "end position =", end_pos

        start_pos = end_pos + (0.010714138798)
        print "start position =", start_pos
        
        calibration = True
        fwd_cnt = 0
        cycle_cnt = 0
        crnt_pos = end_pos
        
        goal = 1
        target = start_pos

        while not rospy.is_shutdown():
            
            if crnt_pos <= target and target == start_pos:
                
                pub.publish(Float64(bck))
                msg = last_state.last()
                crnt_pos = msg.position[0]
                
                rospy.sleep(0.02)
                target = start_pos
                fwd_cnt = 0
                
            elif crnt_pos >= target:
                
                pub.publish(Float64(fwd))
                msg = last_state.last()
                crnt_pos = msg.position[0]
                
                rospy.sleep(0.02)
                target = end_pos  
                fwd_cnt += 1
                
                
                if fwd_cnt > 110:
                    target = start_pos
                    cycle_cnt += 1
                    print "grip count =", cycle_cnt                
                if crnt_pos <= target:
                    target = start_pos
                        
                        
if __name__ == '__main__':
    calibrate()

