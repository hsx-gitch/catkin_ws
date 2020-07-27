/*
Created on Wed Sep 20 11:45:30 2017

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
*/
// author: Cong Liu

#ifndef CYTON_ROS_CONTROL_HW_INTERFACE
#define CYTON_ROS_CONTROL_HW_INTERFACE

#include <ros/ros.h>
#include <urdf/model.h>
#include <pthread.h>
#include <time.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllers.h>

class CuteHWInterface : public hardware_interface::RobotHW
{
public:
    CuteHWInterface();
    void read(const dynamixel_msgs::JointStateConstPtr &msg);
    void publishCmd();
    ros::NodeHandle getnode();

private:
    hardware_interface::JointStateInterface jnt_state_interface;
    hardware_interface::PositionJointInterface jnt_cmd_interface;
    double cmd[8];
    double pos[8];
    double vel[8];
    double eff[8];

    controller_manager_msgs::ListControllersRequest list_req;
    controller_manager_msgs::ListControllersResponse list_resp;

    bool loaded_flag;

    ros::NodeHandle _n;
    ros::Publisher pub_cmd[8];
    std_msgs::Float64 cmd_msg[8];
    ros::Time start_time_;
    ros::Duration start_dur_;

    ros::Subscriber sub_js[8];
    ros::ServiceClient client_controller_list;

};

#endif
