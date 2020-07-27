/*
 * cute_dance.cpp
 *
 *  Created on: 2019.1.26
 *      Author: wxw
 *      Author: wlg
 */
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <boost/thread/thread.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <urdf/model.h>

//cute_dance中增加部分内容

int main(int argc, char **argv) {
  ros::init(argc, argv, "send_goals_node");
  ros::Duration dur(1);
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac(
      "cute_arm_controller/follow_joint_trajectory", true);

  // Wait 60 seconds for the action server to become available
  ROS_INFO("Waiting for the gripper action server");
  ac.waitForServer(ros::Duration(3));
  ROS_INFO("Connected to move base server");
  // Send a goal to move_base1
  control_msgs::FollowJointTrajectoryGoal goal;

  int steps_num = 17;
  float Goalpoint[steps_num][7] = {
      {0.0, 0.006135923151542565, 0.0, 1.603009923340495, 0.0015339807878856412, 0.007669903939428206, 0.0},
      {0.0, 0.0, 0.0030679615757712823, 1.603009923340495, 0.1119805975156518, -1.1397477253990314, 0.0},
      {0.0, 0.0015339807878856412, 0.0030679615757712823, 1.5953400194010667, 0.1119805975156518, 0.8084078752157329, 0.0},
      {0.0, 0.0, 0.0030679615757712823, 1.603009923340495, 0.1119805975156518, -1.1397477253990314, 0.0},
      {0.0, 0.0015339807878856412, 0.0030679615757712823, 1.5953400194010667, 0.1119805975156518, 0.8084078752157329, 0.0},
      {0.0, 0.0, 0.0030679615757712823, 1.603009923340495, 0.1119805975156518, -1.1397477253990314, 0.0},
      {0.0, 0.0015339807878856412, 0.0030679615757712823, 1.5953400194010667, 0.1119805975156518, 0.8084078752157329, 0.0},
      {0.0, 0.0, 0.0030679615757712823, 1.603009923340495, 0.1119805975156518, -1.1397477253990314, 0.0},
      {0.0, 0.0015339807878856412, 0.0030679615757712823, 1.5953400194010667, 0.1119805975156518, 0.8084078752157329, 0.0},
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.43718452454740775, 0.02454369260617026, -0.6872233929727672, 0.007669903939428206, 0.6166602767300278, 0.010737865515199488},
      {0.0, -0.2807184841830723, 0.032213596545598466, 0.6902913545485385, 0.010737865515199488, -0.9035146840646426, 0.018407769454627694},
      {0.0, 0.43718452454740775, 0.02454369260617026, -0.6872233929727672, 0.007669903939428206, 0.6166602767300278, 0.010737865515199488},
      {0.0, -0.2807184841830723, 0.032213596545598466, 0.6902913545485385, 0.010737865515199488, -0.9035146840646426, 0.018407769454627694},
      {0.0, 0.43718452454740775, 0.02454369260617026, -0.6872233929727672, 0.007669903939428206, 0.6166602767300278, 0.010737865515199488},
      {0.0, -0.2807184841830723, 0.032213596545598466, 0.6902913545485385, 0.010737865515199488, -0.9035146840646426, 0.018407769454627694},
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
  };
  goal.trajectory.joint_names.push_back("joint1");
  goal.trajectory.joint_names.push_back("joint2");
  goal.trajectory.joint_names.push_back("joint3");
  goal.trajectory.joint_names.push_back("joint4");
  goal.trajectory.joint_names.push_back("joint5");
  goal.trajectory.joint_names.push_back("joint6");
  goal.trajectory.joint_names.push_back("joint7");
  goal.trajectory.points.resize(steps_num);
  for (int i = 0; i < steps_num; i++) {
    for (int joint = 0; joint < 7; joint++) {
      ROS_INFO("test!!!!!!!!!!!!!!!!!!!");
      goal.trajectory.points[i].positions.resize(7);
      goal.trajectory.points[i].time_from_start = dur.operator*(i + 1);
      goal.trajectory.points[i].positions[joint] = Goalpoint[i][joint];
    }

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    // Wait for the action to return
    ac.waitForResult();
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("You have reached the goal!");
    else
      ROS_INFO("The base failed for some reason");
  }
  return 0;
}