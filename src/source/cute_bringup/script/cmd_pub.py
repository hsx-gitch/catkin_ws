#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 28 14:41:00 2017

@author: Cong Liu
 Software License Agreement (BSD License)

 Copyright (c) 2017, Han's Robot Co., Ltd.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holders nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
 
"""
# author: Cong Liu
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class CmdPub(object):
    def __init__(self):
        self.js_pub=rospy.Publisher('cute_robot/command/joint', JointState, queue_size=1)
        self.ps_pub=rospy.Publisher('cute_robot/command/pose', PoseStamped, queue_size=1)
    def function_pub_js(self):
        js=JointState()
        js.name=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        js.position=[0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
        js.header.stamp=rospy.get_rostime()
        self.js_pub.publish(js)

    def function_pub_ps(self):
        ps=PoseStamped()
        ps.header.stamp=rospy.get_rostime()
        ps.header.frame_id='world'
        ps.pose.position.x=-0.194776
        ps.pose.position.y=0.0921016
        ps.pose.position.z=0.171236
        ps.pose.orientation.x=0.978937
        ps.pose.orientation.y=-0.176944
        ps.pose.orientation.z=-0.101733
        ps.pose.orientation.w=-0.00476077
        self.ps_pub.publish(ps)

if __name__=='__main__':
    rospy.init_node('cmd_pub', anonymous=True)
    cp=CmdPub()
    rospy.sleep(1)
    # cp.function_pub_js()
    cp.function_pub_ps()

    rospy.spin()
