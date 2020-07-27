#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Aug 19 11:13:35 2017

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
import threading
from sensor_msgs.msg import JointState as sensor_JS
from dynamixel_msgs.msg import JointState as dyn_JS

class DynJS_Manager(object):
    def __init__(self, revolute_controllers=[], prismatic_controllers=[],
                 prismatic_factors=[]):
        self.r_controllers=revolute_controllers
        self.p_controllers=prismatic_controllers
        self.p_factors=prismatic_factors
        self.controllers=self.r_controllers+self.p_controllers
        for i in xrange(len(self.controllers)):
            rospy.Subscriber(self.controllers[i]+'_controller/state', dyn_JS,
                             self.dyn_js_cb, i,)
        self.js_name=['' for i in xrange(len(self.controllers))]
        self.js_position=[0 for i in xrange(len(self.controllers))]
        self.js_velocity=[0 for i in xrange(len(self.controllers))]
        self.js_effort=[0 for i in xrange(len(self.controllers))]
        
        self.js_pub=rospy.Publisher('joint_states', sensor_JS, queue_size=1)
        self.js_msg=sensor_JS()
        
        self.simple_lock=threading.Lock()
        
    def dyn_js_cb(self, data, serial_num):
        if serial_num>=len(self.r_controllers):
            p_serial_num=serial_num-len(self.r_controllers)
            if len(self.p_factors)>p_serial_num and \
               len(self.p_factors[p_serial_num])==2:
                if self.simple_lock.acquire():
                    self.js_name[serial_num]=data.name
                    self.js_position[serial_num]=self.p_factors[p_serial_num][0]*data.current_pos\
                                                 +self.p_factors[p_serial_num][1]
                    self.js_velocity[serial_num]=self.p_factors[p_serial_num][0]*data.velocity
                    self.js_effort[serial_num]=data.load
                    self.simple_lock.release()
            else:
                rospy.logerr('the parameter prismatic_factors is set wrong')
        else:
            if self.simple_lock.acquire():
                self.js_name[serial_num]=data.name
                self.js_position[serial_num]=data.current_pos
                self.js_velocity[serial_num]=data.velocity
                self.js_effort[serial_num]=data.load
                self.simple_lock.release()
    
    def joint_states_publish(self):
        self.js_msg.header.stamp=rospy.get_rostime()
        if self.simple_lock.acquire():
            self.js_msg.name=self.js_name
            self.js_msg.position=self.js_position
            self.js_msg.velocity=self.js_velocity
            self.js_msg.effort=self.js_effort
            self.simple_lock.release()
        
        self.js_pub.publish(self.js_msg)
        
    def loop(self):
        r=rospy.Rate(20)
        while not rospy.is_shutdown():
            self.joint_states_publish()
            r.sleep()
    
if __name__ == '__main__':
    rospy.init_node('dynamixel_joint_state_manager', anonymous=True)

    revol_controllers=rospy.get_param('revolute_controllers', [])
    pris_controllers=rospy.get_param('prismatic_controllers', [])
    pris_factors=rospy.get_param('prismatic_factors', [])
    
    dyn_js_manager=DynJS_Manager(revol_controllers, pris_controllers, 
                                 pris_factors)
    rospy.sleep(1)
    dyn_js_manager.loop()

