#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 28 09:48:30 2017

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
import math
import moveit_commander
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv._GetPositionIK import GetPositionIK, GetPositionIKRequest

class CuteBasicAPI(object):
    def __init__(self):
        self.robot=moveit_commander.RobotCommander()
        self.scene=moveit_commander.PlanningSceneInterface()
        self.group=moveit_commander.MoveGroupCommander('cute_arm')
        self.group.set_end_effector_link('end_link')
        
        self.joint_state=JointState()
        self.joint_limit_max=[0.58*math.pi]*7
        self.joint_limit_min=[-0.58*math.pi]*7
        self.joint_names=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        rospy.Subscriber('cute_robot/command/joint', JointState, self.command_joint_cb)
        rospy.Subscriber('cute_robot/command/pose', PoseStamped, self.command_pose_cb)
        rospy.Subscriber('joint_states', JointState, self.joint_state_cb)
        
        self.call_ik=rospy.ServiceProxy('compute_ik', GetPositionIK)
        self.call_request=GetPositionIKRequest()
        self.request_init()
        
    def request_init(self):
        self.call_request.ik_request.group_name=self.group.get_name()
        self.call_request.ik_request.robot_state=self.robot.get_current_state()
        self.call_request.ik_request.ik_link_name='end_link'
        
    def command_joint_cb(self, data):
        self.group.go(joints=data)
        
    def command_pose_cb(self, data):
#==============================================================================
#         self.call_request.ik_request.pose_stamped=data
#         js_cmd=self.call_ik.call(self.call_request)
#         self.group.go(joints=js_cmd.solution.joint_state)
#==============================================================================
        self.group.go(data)
        
    def joint_state_cb(self, data):
        self.joint_state=data

if __name__=='__main__':
    rospy.init_node('cute_basic_api', anonymous=True)
    cba=CuteBasicAPI()
    rospy.spin()

