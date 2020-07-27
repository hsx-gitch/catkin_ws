/*
Created on Wed Sep 20 11:30:15 2017

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

#include "cute_ros_control/cute_hardware_interface.h"

boost::mutex cute_io_mutex;

CuteHWInterface::CuteHWInterface()
{
    for(int i=0; i<7; i++)
    {
        std::string joint_cmd_name="joint";
        std::string joint_state_name="joint";
        std::string joint_num=boost::lexical_cast<std::string>(i+1);
        joint_cmd_name.append(joint_num);
        joint_state_name.append(joint_num);
        joint_cmd_name.append("_controller/command");
        joint_state_name.append("_controller/state");
        pub_cmd[i]=_n.advertise<std_msgs::Float64>(joint_cmd_name, 1);
        sub_js[i]=_n.subscribe(joint_state_name, 1, &CuteHWInterface::read, this);
        client_controller_list=_n.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");
        loaded_flag=false;
    }

    for(int i=0; i<7; i++)
    {
        std::string joint_name="joint";
        std::string joint_num=boost::lexical_cast<std::string>(i+1);
        joint_name.append(joint_num);
        hardware_interface::JointStateHandle jnt_state_handle_tmp(joint_name, &pos[i], &vel[i], &eff[i]);
        jnt_state_interface.registerHandle(jnt_state_handle_tmp);
    }

    registerInterface(&jnt_state_interface);

    for(int i=0; i<7; i++)
    {
        std::string joint_name="joint";
        std::string joint_num=boost::lexical_cast<std::string>(i+1);
        joint_name.append(joint_num);
        hardware_interface::JointHandle jnt_handle_tmp(jnt_state_interface.getHandle(joint_name), &cmd[i]);
        jnt_cmd_interface.registerHandle(jnt_handle_tmp);
    }

    registerInterface(&jnt_cmd_interface);

    start_time_=ros::Time::now();
    start_dur_.operator +=(ros::Duration(3));
}

void CuteHWInterface::publishCmd()
{
    if(ros::Time::now()-start_time_<start_dur_)
        return;
    for(int i=0; i<7; i++)
    {
        cmd_msg[i].data=cmd[i];
        pub_cmd[i].publish(cmd_msg[i]);
    }
}

void CuteHWInterface::read(const dynamixel_msgs::JointStateConstPtr &msg)
{
    if(msg->motor_ids.size()<=0)
    {
        return;
    }
    if(msg->motor_ids[0]>6 || msg->motor_ids[0]<0)
    {
        return;
    }
    int msg_num=msg->motor_ids[0];
    double bm=msg->current_pos-pos[msg_num];
    boost::mutex::scoped_lock lock(cute_io_mutex);
    pos[msg_num]=msg->current_pos;
    if(ros::Time::now()-start_time_>start_dur_)
    {
        if(bm>=0)
            vel[msg_num]=msg->velocity;
        else
            vel[msg_num]=-1*msg->velocity;
    }
    else
        vel[msg_num]=0;

    if(fabs(vel[msg_num])<1.2)
        vel[msg_num]=0;

    eff[msg_num]=msg->load;
}

ros::NodeHandle CuteHWInterface::getnode()
{
    return _n;
}

static void timespecInc(struct timespec &tick, int nsec)
{
  int SEC_2_NSEC = 1e+9;
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= SEC_2_NSEC)
  {
    tick.tv_nsec -= SEC_2_NSEC;
    ++tick.tv_sec;
  }
}

void* update_loop(void* threadarg)
{
    controller_manager::ControllerManager *c=(controller_manager::ControllerManager *)threadarg;
    ros::Rate r(50);
    ros::Duration d(0.02);

    while(ros::ok())
    {
        boost::mutex::scoped_lock lock(cute_io_mutex);
        c->update(ros::Time::now(), d);
        lock.unlock();
        r.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"cute_hardware_interface", ros::init_options::AnonymousName);
    CuteHWInterface c1;

    // init joint states
    ros::Rate r_tmp(10);
    for(int i=0; i<50; i++)
    {
        ros::spinOnce();
        r_tmp.sleep();
        if(i%10==0)
            ROS_INFO("cute bring up in %i", (10-i/10));
    }

    controller_manager::ControllerManager cm(&c1);
    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, (void *)&cm);

    for(int i=0; i<50; i++)
    {
        ros::spinOnce();
        r_tmp.sleep();
        if(i%10==0)
            ROS_INFO("cute bring up in %i", (5-i/10));
    }
    ROS_INFO("cute bring up successfully");
    // loop
    ros::Rate r(50);
    while(ros::ok())
    {
        boost::mutex::scoped_lock lock(cute_io_mutex);
        c1.publishCmd();
        lock.unlock();
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
