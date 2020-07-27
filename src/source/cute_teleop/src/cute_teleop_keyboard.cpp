/*
Created on Wed Aug 23 10:40:20 2017

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
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <vector>

#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <urdf/model.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#define _USE_MATH_DEFINES

#define KEYCODE_X 0x78
#define KEYCODE_C 0x63
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_V 0x76
#define KEYCODE_B 0x62

#define KEYCODE_N 0x6e
#define KEYCODE_M 0x6d
#define KEYCODE_J 0x6a
#define KEYCODE_K 0x6b
#define KEYCODE_U 0x75
#define KEYCODE_I 0x69
#define KEYCODE_comma 0x2c
#define KEYCODE_dot 0x2e

sensor_msgs::JointState js_current;
double claw_state;

std::vector<double> angular_limit_upper;
std::vector<double> angular_limit_lower;

bool flag_discontinuous_movement=false;

moveit_msgs::GetPositionFK::Request request_fk;
moveit_msgs::GetPositionFK::Response response_fk;
ros::ServiceClient client_fk;

moveit_msgs::GetPositionIK::Request request_ik;
moveit_msgs::GetPositionIK::Response response_ik;
ros::ServiceClient client_ik;

void getAngularLimit()
{
    std::string rbt_name="robot_description";
    urdf::Model urdf_model;
    if(!urdf_model.initParam(rbt_name))
    {
        ROS_ERROR("Can't get %s", rbt_name.c_str());
        exit(0);
    }
    std::string joint_names[7]={"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    angular_limit_upper.clear();
    angular_limit_lower.clear();

    for(int i=0; i<7; i++)
    {
        double limit_upper_tmp=urdf_model.getJoint(joint_names[i])->limits->upper;
        double limit_lower_tmp=urdf_model.getJoint(joint_names[i])->limits->lower;

        angular_limit_upper.push_back(limit_upper_tmp);
        angular_limit_lower.push_back(limit_lower_tmp);
    }
}

void getJointStates(const sensor_msgs::JointState::ConstPtr& msg)
{
    js_current.position.clear();
    std::string joint_names[]={"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    std::vector<std::basic_string<char> >::const_iterator it;
    for(int i=0; i<7; i++)
    {
        it=std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
        if(it!=msg->name.end())
        {
            int name_position=std::distance(msg->name.begin(), it);
            js_current.position.push_back(msg->position[name_position]);
        }
        else
        {
            js_current.position.push_back(0);
        }
    }

    it=std::find(msg->name.begin(), msg->name.end(), "claw");
    if(it!=msg->name.end())
    {
        int claw_name_position=std::distance(msg->name.begin(), it);
        claw_state=msg->position[claw_name_position];
    }
    else
    {
        claw_state=0;
    }
}

class CuteKeyboardTeleopNode
{
    private:
        
        std_msgs::Float64 msg_claw;
        ros::NodeHandle n;
        ros::Publisher pub_claw;
        bool flag_joint_space;

    public:
        CuteKeyboardTeleopNode()
        {
            pub_claw = n.advertise<std_msgs::Float64>("/claw_controller/command", 1);
            flag_joint_space=true;
        }
                
        ~CuteKeyboardTeleopNode() { }
        void keyboardLoop(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &aclient);
       
};

CuteKeyboardTeleopNode** cute_teleop;
int kfd = 0;
struct termios cooked, raw;
bool done;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"cute_teleop_keyboard", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    CuteKeyboardTeleopNode cute_teleop;
    
    ros::NodeHandle n_;

    getAngularLimit();
    
    ros::Subscriber sub_js=n_.subscribe("/joint_states", 1, getJointStates);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("cute_arm_controller/follow_joint_trajectory", true);
    client_fk=n_.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
    client_ik=n_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");

    std::string name_array[]={"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    for(int i=0; i<7; i++)
    {
        js_current.name.push_back(name_array[i]);
    }

    request_fk.fk_link_names.push_back("end_link");
    request_ik.ik_request.ik_link_name="end_link";
    request_ik.ik_request.group_name="cute_arm";

    puts("Reading from keyboard");
    puts("u: continuous movement  i: discontinuous movement");
    puts("j: teleop in joint space   k: teleop in cart.-coor.-system");
    puts("x&c for joint1             x&c for x axis");
    puts("s&d for joint2             s&d for y axis");
    puts("w&e for joint3             w&e for z axis");
    puts("r&t for joint4             r&t for yaw");
    puts("f&g for joint5             f&g for pitch");
    puts("v&b for joint6             v&b for roll");
    puts("n&m for joint7             ,&. for claw");
    puts(",&. for claw");
    puts("now teleop in joint space, continuous movement");

    ros::Rate r(10);
    while(ros::ok())
    {
        cute_teleop.keyboardLoop(ac);
        tcflush(kfd, TCIFLUSH); // clean up the input memery
        ros::spinOnce();
        r.sleep();
    }

    
    return(0);
}

void sendGoal(const int &joint_num,
              actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &a_client)
{
//    a_client.cancelAllGoals();
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("joint1");
    goal.trajectory.joint_names.push_back("joint2");
    goal.trajectory.joint_names.push_back("joint3");
    goal.trajectory.joint_names.push_back("joint4");
    goal.trajectory.joint_names.push_back("joint5");
    goal.trajectory.joint_names.push_back("joint6");
    goal.trajectory.joint_names.push_back("joint7");
    if(abs(joint_num)<20)
    {
        double resolution=0.03;
        ros::Duration dur(0.1);
        int direction=joint_num/abs(joint_num);
        int steps_num;
        if(direction>0)
        {
            steps_num=int((angular_limit_upper[abs(joint_num)-1]-js_current.position[abs(joint_num)-1])/resolution);
        }
        else
        {
            steps_num=int((js_current.position[abs(joint_num)-1]-angular_limit_lower[abs(joint_num)-1])/resolution);
        }
        if(steps_num<=0)
            steps_num=1;
        if(steps_num>1 && flag_discontinuous_movement)
            steps_num=2;
        goal.trajectory.points.resize(steps_num);
        for(int i=0; i<steps_num; i++)
        {
            goal.trajectory.points[i].positions=js_current.position;
            goal.trajectory.points[i].time_from_start=dur.operator *(i+1);
            goal.trajectory.points[i].positions[abs(joint_num)-1]+=i*resolution*joint_num/abs(joint_num);
        }
        a_client.sendGoal(goal);
        if(flag_discontinuous_movement)
            a_client.waitForResult(ros::Duration(2));
    }
    else
    {
        request_fk.robot_state.joint_state=js_current;
        client_fk.call(request_fk, response_fk);
        if(abs(joint_num)<27)
        {
            geometry_msgs::Pose begin_pose=response_fk.pose_stamped[0].pose;
            geometry_msgs::Pose pose_tmp=begin_pose;
            sensor_msgs::JointState js_tmp=js_current;
            int error_code=1;
            int times=0;
            double resolution_l=0.005;
            double resolution_a=0.02;
            ros::Duration dur_l(0.3);
            ros::Duration dur_a(0.3);
            int max_times;
            if(flag_discontinuous_movement)
                max_times=2;
            else
                max_times=30;
            while(error_code==1 && times<max_times)
            {                
                if(times>0)
                {
                    double max_distance=0;
                    double big_times=0;
                    for(int i=0; i<7; i++)
                    {
                        if(fabs(goal.trajectory.points[times-1].positions[i]-js_tmp.position[i])>max_distance)
                            max_distance=fabs(goal.trajectory.points[times-1].positions[i]-js_tmp.position[i]);
                        if(fabs(goal.trajectory.points[times-1].positions[i]-js_tmp.position[i])>0.2)
                            big_times++;
                    }
                    if(max_distance>0.5 || big_times>2)
                        break;
                }

                goal.trajectory.points.resize(times+1);
                goal.trajectory.points[times].positions=js_tmp.position;
                if(abs(joint_num)<24)
                {
                    goal.trajectory.points[times].time_from_start=dur_l.operator *(times+1);
                }
                else
                {
                    goal.trajectory.points[times].time_from_start=dur_a.operator *(times+1);
                }

                request_ik.ik_request.robot_state.joint_state=js_tmp;

                if(abs(joint_num)==21)
                {
                    pose_tmp.position.x+=resolution_l*joint_num/abs(joint_num);
                }
                if(abs(joint_num)==22)
                {
                    pose_tmp.position.y+=resolution_l*joint_num/abs(joint_num);
                }
                if(abs(joint_num)==23)
                {
                    pose_tmp.position.z+=resolution_l*joint_num/abs(joint_num);
                }
                if(abs(joint_num)==24)
                {
                    tf::Quaternion q_1(pose_tmp.orientation.x, pose_tmp.orientation.y,
                                                 pose_tmp.orientation.z, pose_tmp.orientation.w);
                    tf::Vector3 x_axis(1, 0, 0);
                    tf::Quaternion q_2(x_axis, resolution_a*joint_num/abs(joint_num));
                    tf::Matrix3x3 m(q_1);
                    tf::Matrix3x3 m_2(q_2);
                    m.operator *=(m_2);
                    double r, p, y;
                    m.getRPY(r,p,y);
//                    r+=resolution_a*joint_num/abs(joint_num);
//                    tf::Quaternion q_2;
                    q_2.setRPY(r, p, y);
                    pose_tmp.orientation.x=q_2.getX();
                    pose_tmp.orientation.y=q_2.getY();
                    pose_tmp.orientation.z=q_2.getZ();
                    pose_tmp.orientation.w=q_2.getW();
                }
                if(abs(joint_num)==25)
                {
                    tf::Quaternion q_1(pose_tmp.orientation.x, pose_tmp.orientation.y,
                                                 pose_tmp.orientation.z, pose_tmp.orientation.w);
                    tf::Vector3 y_axis(0, 1, 0);
                    tf::Quaternion q_2(y_axis, resolution_a*joint_num/abs(joint_num));
                    tf::Matrix3x3 m(q_1);
                    tf::Matrix3x3 m_2(q_2);
                    m.operator *=(m_2);
                    double r, p, y;
                    m.getRPY(r,p,y);
//                    p+=resolution_a*joint_num/abs(joint_num);
//                    tf::Quaternion q_2;
                    q_2.setRPY(r, p, y);
                    pose_tmp.orientation.x=q_2.getX();
                    pose_tmp.orientation.y=q_2.getY();
                    pose_tmp.orientation.z=q_2.getZ();
                    pose_tmp.orientation.w=q_2.getW();
                }
                if(abs(joint_num)==26)
                {
                    tf::Quaternion q_1(pose_tmp.orientation.x, pose_tmp.orientation.y,
                                                 pose_tmp.orientation.z, pose_tmp.orientation.w);
                    tf::Vector3 z_axis(0, 0, 1);
                    tf::Quaternion q_2(z_axis, resolution_a*joint_num/abs(joint_num));
                    tf::Matrix3x3 m(q_1);
                    tf::Matrix3x3 m_2(q_2);
                    m.operator *=(m_2);
                    double r, p, y;
                    m.getRPY(r,p,y);
//                    y+=resolution_a*joint_num/abs(joint_num);
//                    tf::Quaternion q_2;
                    q_2.setRPY(r, p, y);
                    pose_tmp.orientation.x=q_2.getX();
                    pose_tmp.orientation.y=q_2.getY();
                    pose_tmp.orientation.z=q_2.getZ();
                    pose_tmp.orientation.w=q_2.getW();
                }
                request_ik.ik_request.pose_stamped.pose=pose_tmp;
                client_ik.call(request_ik, response_ik);
                js_tmp.position.clear();
                std::string joint_names[]={"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
                std::vector<std::basic_string<char> >::iterator it;
                for(int i=0; i<7; i++)
                {
                    it=std::find(response_ik.solution.joint_state.name.begin(),
                                        response_ik.solution.joint_state.name.end(), joint_names[i]);
                    if(it!=response_ik.solution.joint_state.name.end())
                    {
                        int name_position=std::distance(response_ik.solution.joint_state.name.begin(), it);
                        js_tmp.position.push_back(response_ik.solution.joint_state.position[name_position]);
                    }
                    else
                    {
                        js_tmp.position.push_back(0);
                    }
                }
                error_code=response_ik.error_code.val;
                times++;
            }
            a_client.sendGoal(goal);
            if(flag_discontinuous_movement)
                a_client.waitForResult(ros::Duration(2));
        }
    }

}

void CuteKeyboardTeleopNode::keyboardLoop(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>& aclient)
{
    char c;
    static bool dirty = false;
    static int smooth_code=0;
    
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memset(&raw, 0, sizeof(struct termios));
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;
    
    for(int i=0; i<1; i++)
    {
//        boost::this_thread::interruption_point();
        
        // get the next event from the keyboard
        int num;
        
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        else
        {
//            if (dirty == true)
//            {
//                dirty = false;
//            }
            
//            continue;
        }
        
        switch(c)
        {
            case KEYCODE_X:
                if(flag_joint_space)
                {
                    if(js_current.position[0] < angular_limit_upper[0] && dirty==false)
                    {
                        sendGoal(1, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else
                {
                    if(dirty==false)
                    {
                        sendGoal(21, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
            case KEYCODE_C:
                if(flag_joint_space)
                {
                    if(js_current.position[0] > angular_limit_lower[0] && dirty==false)
                    {
                        sendGoal(-1, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else
                {
                    if(dirty==false)
                    {
                        sendGoal(-21, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
            case KEYCODE_S:
                if(flag_joint_space)
                {
                    if (js_current.position[1]<angular_limit_upper[1]&& dirty==false)
                    {
                        sendGoal(2, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else
                {
                    if(dirty==false)
                    {
                        sendGoal(22, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
            case KEYCODE_D:
                if(flag_joint_space)
                {
                    if (js_current.position[1]>angular_limit_lower[1]&& dirty==false)
                    {
                        sendGoal(-2, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else
                {
                    if(dirty==false)
                    {
                        sendGoal(-22, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
            case KEYCODE_W:
                if(flag_joint_space)
                {
                    if (js_current.position[2]<angular_limit_upper[2]&& dirty==false)
                    {
                        sendGoal(3, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else
                {
                    if(dirty==false)
                    {
                        sendGoal(23, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
            case KEYCODE_E:
                if(flag_joint_space)
                {
                    if (js_current.position[2]>angular_limit_lower[2]&& dirty==false)
                    {
                        sendGoal(-3, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else
                {
                    if(dirty==false)
                    {
                        sendGoal(-23, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
            case KEYCODE_R:
                if(flag_joint_space)
                {
                    if (js_current.position[3]<angular_limit_upper[3]&& dirty==false)
                    {
                        sendGoal(4, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else
                {
                    if(dirty==false)
                    {
                        sendGoal(26, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
            case KEYCODE_T:
                if(flag_joint_space)
                {
                    if (js_current.position[3]>angular_limit_lower[3]&& dirty==false)
                    {
                        sendGoal(-4, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else
                {
                    if(dirty==false)
                    {
                        sendGoal(-26, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
            case KEYCODE_F:
                if(flag_joint_space)
                {
                    if (js_current.position[4]<angular_limit_upper[4]&& dirty==false)
                    {
                        sendGoal(5, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else
                {
                    if(dirty==false)
                    {
                        sendGoal(25, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
            case KEYCODE_G:
                if(flag_joint_space)
                {
                    if (js_current.position[4]>angular_limit_lower[4]&& dirty==false)
                    {
                        sendGoal(-5, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else
                {
                    if(dirty==false)
                    {
                        sendGoal(-25, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
            case KEYCODE_V:
                if(flag_joint_space)
                {
                    if (js_current.position[5]<angular_limit_upper[5]&& dirty==false)
                    {
                        sendGoal(6, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else
                {
                    if(dirty==false)
                    {
                        sendGoal(24, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
            case KEYCODE_B:
                if(flag_joint_space)
                {
                    if (js_current.position[5]>angular_limit_lower[5]&& dirty==false)
                    {
                        sendGoal(-6, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else
                {
                    if(dirty==false)
                    {
                        sendGoal(-24, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
            case KEYCODE_N:
                if(flag_joint_space)
                {
                    if (js_current.position[6]<angular_limit_upper[6]&& dirty==false)
                    {
                        sendGoal(7, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else break;
            case KEYCODE_M:
                if(flag_joint_space)
                {
                    if (js_current.position[6]>angular_limit_lower[6]&& dirty==false)
                    {
                        sendGoal(-7, aclient);
                        if(!flag_discontinuous_movement)
                            dirty = true;
                        break;
                    }
                    else break;
                }
                else break;
            case KEYCODE_comma:
                if(claw_state<0.01-0.001)
                {
                    msg_claw.data=(claw_state+0.001+0.005)/(-0.0072);
                    pub_claw.publish(msg_claw);
                    break;
                }
                else break;
            case KEYCODE_dot:
                if(claw_state>-0.005+0.001)
                {
                    msg_claw.data=(claw_state-0.001+0.005)/(-0.0072);
                    pub_claw.publish(msg_claw);
                    break;
                }
                else break;
            case KEYCODE_J:
                flag_joint_space=true;
                puts("j: now teleop in joint space");
                break;
            case KEYCODE_K:
                flag_joint_space=false;
                puts("k: now teleop in cartesian coordination system");
                break;
            case KEYCODE_U:
                flag_discontinuous_movement=false;
                puts("u: continuous movement");
                break;
            case KEYCODE_I:
                flag_discontinuous_movement=true;
                puts("i: discontinuous movement");
                break;
            default:
                if(dirty && smooth_code>=2)
                {
                    aclient.cancelAllGoals();
                    dirty = false;
                    smooth_code=0;
                    break;
                }
                if(dirty && smooth_code<2)
                {
                    smooth_code++;
                    break;
                }
        }
    }

}


