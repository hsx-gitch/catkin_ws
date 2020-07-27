#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import Constraints, OrientationConstraint, PositionConstraint
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class MoveItConstraintsDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('arm')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'cute_base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.set_planning_time(15)
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.1)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(2)
               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.127285
        target_pose.pose.position.y = 0.156488
        target_pose.pose.position.z = 0.546645
        target_pose.pose.orientation.x = 0.707112
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0.707111
        target_pose.pose.orientation.w = 0
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)

        # 创建约束条件
        constraints = Constraints()
        constraints.name = "Keep gripper horizontal"
        
        # 设置约束参数
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = target_pose.header
        orientation_constraint.link_name = arm.get_end_effector_link()
        orientation_constraint.orientation.x = 0.707112
        orientation_constraint.orientation.y = 0
        orientation_constraint.orientation.z = 0.707111
        orientation_constraint.orientation.w = 0
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 3.14
        orientation_constraint.weight = 1.0
        
        constraints.orientation_constraints.append(orientation_constraint)
          
        # 将约束条件设置到规划组当中
        arm.set_path_constraints(constraints)
        
        # 规划运动路径
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)

        # 清除规划组中的约束条件
        arm.clear_path_constraints()
           
        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItConstraintsDemo()

    
    
