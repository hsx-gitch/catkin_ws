#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 14 17:41:06 2017

@author: Cong Liu
"""
import rospy
from std_msgs.msg import Float64

class ClawCmdPub(object):
    def __init__(self):
        self.cmd_pub_1=rospy.Publisher('claw_controller_1/command', Float64, queue_size=1)
        self.cmd_pub_2=rospy.Publisher('claw_controller_2/command', Float64, queue_size=1)
        self.cmd=Float64()
    
    def claw_cmd_cb(self, data):        
        self.cmd.data=data.data*-0.0072-0.005
        self.cmd_pub_1.publish(self.cmd)
        self.cmd_pub_2.publish(self.cmd)
    
    def listen(self):
        rospy.Subscriber('claw_controller/command', Float64, self.claw_cmd_cb)

if __name__ == '__main__':
    rospy.init_node('claw_cmd_pub', anonymous=True)
    ccp=ClawCmdPub()
    ccp.listen()
    rospy.spin()
