#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 15 16:39:38 2017

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
from std_msgs.msg import Float64
from dynamixel_controllers.srv import TorqueEnable, TorqueEnableRequest
from dynamixel_controllers.srv import SetSpeed, SetSpeedRequest, SetSpeedResponse
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from trajectory_msgs.msg import JointTrajectoryPoint
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

class RobotArmDynManager(object):
    def __init__(self, arm_name=''):
        self.controller_names=rospy.get_param(arm_name+'_controller_names', [])
        self.go_home_time=rospy.get_param(arm_name+'_go_home_time', 10)
        if self.go_home_time<2:
            self.go_home_time=2
        rospy.Service(arm_name+'_torque_enable', SetBool, self.torque_enable_cb)
        rospy.Service(arm_name+'_go_home', SetBool, self.go_home_cb)
        rospy.Service(arm_name+'_set_speed', SetSpeed, self.set_speed_cb)
        self.action_client=SimpleActionClient('cute_arm_controller/follow_joint_trajectory',
                                              FollowJointTrajectoryAction)
        self.action_goal=FollowJointTrajectoryGoal()
        self.action_goal.trajectory.joint_names=self.controller_names
        self.pubs=[]
        for i in xrange(len(self.controller_names)):
            pub_tmp=rospy.Publisher(self.controller_names[i]+'_controller/command',
                                    Float64, queue_size=1)
            self.pubs.append(pub_tmp)

    def torque_enable_cb(self, req):
        resp=SetBoolResponse()
        torque_require=TorqueEnableRequest()
        torque_require.torque_enable=req.data
        
        # Send empty trajectory
        if req.data==True:
            self.action_client.wait_for_server()
            self.action_goal.trajectory.header.stamp.secs=0
            self.action_goal.trajectory.header.stamp.nsecs=0
            self.action_goal.trajectory.points=[]
            self.action_client.send_goal(self.action_goal)
            rospy.sleep(1)
        
        for i in xrange(len(self.controller_names)):
            cl_tmp=rospy.ServiceProxy(self.controller_names[i]+'_controller/torque_enable',
                                      TorqueEnable)
            cl_tmp.call(torque_require)
            rospy.sleep(0.3)
        resp.success=True
        return resp
    
    def go_home_cb(self, req):
        resp=SetBoolResponse()
        torque_require=SetBoolRequest()
        torque_require.data=True
        self.torque_enable_cb(torque_require)
        
        self.action_client.wait_for_server()
        self.action_goal.trajectory.header.stamp.secs=0
        self.action_goal.trajectory.header.stamp.nsecs=0
        
        point_tmp=JointTrajectoryPoint()
        position_tmp=[]
        for i in xrange(len(self.controller_names)):
            position_tmp.append(0)
        point_tmp.positions=position_tmp
        point_tmp.time_from_start.secs=int(self.go_home_time)
        print len(self.action_goal.trajectory.points)
        self.action_goal.trajectory.points.append(point_tmp)
        self.action_client.send_goal(self.action_goal)
        self.action_client.wait_for_result()
        self.action_goal.trajectory.points=[]

        resp.success=True
        return resp
            
    
    def set_speed_cb(self, req):
        resp=SetSpeedResponse()
        for i in xrange(len(self.controller_names)):
            cl_tmp=rospy.ServiceProxy(self.controller_names[i]+'_controller/set_speed',
                                      SetSpeed)
            cl_tmp.call(req)
            rospy.sleep(0.1)
        return resp
    
    def joint_command_cb(self, data):
        if len(data.data)!=len(self.controller_names):
            rospy.logerr('length of the command_joint is wrong')
            return
        speed_require=SetSpeedRequest()
        speed_require.speed=0.3
        self.set_speed_cb(speed_require)
        for i in xrange(len(self.controller_names)):
            self.pubs[i].publish(data.data[i])

if __name__ == '__main__':
    rospy.init_node('robot_arm_dynamixel_manager', anonymous=True)
    arm_names=rospy.get_param('robot_arm_names', [])
    managers=[None]*len(arm_names)
    for i in xrange(len(arm_names)):
        managers[i]=RobotArmDynManager(arm_names[i])
    rospy.spin()
