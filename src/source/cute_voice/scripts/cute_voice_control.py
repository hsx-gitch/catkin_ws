#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import roslib
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import moveit_commander
from control_msgs.msg import GripperCommand
from std_srvs.srv import SetBool

class voice_cmd_vel:

    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        #rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        #arm = moveit_commander.MoveGroupCommander('cute_arm')
        
        # 初始化需要使用move group控制的机械臂中的gripper group
        #gripper = moveit_commander.MoveGroupCommander('cute_gripper')
        
        # 设置机械臂和夹爪的允许误差值
        arm.set_goal_joint_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.001)

        #rospy.on_shutdown(self.cleanup)
        #self.speed = 0.4
        #self.msg = String()
        #self.msg.data = "Wake Up!"
	
        # publish to cmd_vel, subscribe to speech output
        #self.pub_ = rospy.Publisher('/voiceWakeup', String, queue_size=10)
        rospy.Subscriber('/voiceWords', String, self.speechCb)

        # rospy.wait_for_service('/cute_go_home')

        # try:
        #     go_home = rospy.ServiceProxy('/cute_go_home', SetBool)
        #     go_home(True)
        # except rospy.ServiceException, e:
        #     #print "Service call failed: %s"%e
        #     rospy.loginfo('Service /cute_go_home call failed: %s' %e)

        #r = rospy.Rate(10.0)
        #while not rospy.is_shutdown():
        #self.pub_.publish(self.msg)
            #r.sleep()
        rospy.spin()
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)
        #rospy.loginfo(self.speed)
	
        if msg.data.find("向前") > -1:
            # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
            joint_positions = [0, 0, 0, 1.5, 0, 1.5, 1.5]
            arm.set_joint_value_target(joint_positions)
                    
            # 控制机械臂完成运动
            arm.go()
            rospy.sleep(1)
        elif msg.data.find("向后") > -1:
            # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
            joint_positions = [0, 0, 0, -1.5, 0, -1.5, -1.5]
            arm.set_joint_value_target(joint_positions)

            # 控制机械臂完成运动
            arm.go()
            rospy.sleep(1)
        elif msg.data.find("向左") > -1:
            # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
            joint_positions = [1.5, 0.5, 0, -1.5, 0, -1.5, -1.5]
            arm.set_joint_value_target(joint_positions)

            # 控制机械臂完成运动
            arm.go()
            rospy.sleep(1)
        elif msg.data.find("向右") > -1:
            # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
            joint_positions = [-1.5, 0.5, 0, -1.5, 0, -1.5, -1.5]
            arm.set_joint_value_target(joint_positions)

            # 控制机械臂完成运动
            arm.go()
            rospy.sleep(1)
        elif msg.data.find("立正") > -1:
            # 控制机械臂先回到初始化位置
            arm.set_named_target('home')
            arm.go()
            rospy.sleep(2)
        elif msg.data.find("停止") > -1 or msg.data.find("停") > -1:          
            #self.msg = String()
            # 关闭并退出moveit
            moveit_commander.roscpp_shutdown()
            moveit_commander.os._exit(0)
        
        #self.pub_.publish(self.msg)

    #def cleanup(self):
        # stop the robot!
        #twist = Twist()
        #self.pub_.publish(twist)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    # 初始化需要使用move group控制的机械臂中的arm group
    arm = moveit_commander.MoveGroupCommander('cute_arm')
    
    # 初始化需要使用move group控制的机械臂中的gripper group
    gripper = moveit_commander.MoveGroupCommander('cute_gripper')
    try:
        voice_cmd_vel()
    except:
        pass

