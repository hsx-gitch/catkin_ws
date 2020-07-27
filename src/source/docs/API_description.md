API 简介
======
if you don't speak chinese, please [click here](./API_description_english.md).

### Subscribed Topics:

* **claw_controller/command (std_msgs/Float64)**  
控制夹爪电机的转动，取值范围：0（打开）～-2.0（闭合）

* **cute_robot/command/joint (sensor_msgs/JointState)**  
令机械臂规划到达指定臂型的路径并执行此路径  
example: function_pub_js() in /cute_bringup/script/cmd_pub.py

* **cute_robot/command/pose (geometry_msgs/PoseStamped)**  
令机械臂规划到达指定空间位置的路径并执行此路径  
example: function_pub_ps in /cute_bringup/script/cmd_pub.py

* **cute_arm_controller/command (trajectory_msgs/JointTrajectory)**  
本消息内容为一条轨迹，可令机械臂沿着这条轨迹运动。

* **cute_arm_controller/follow_joint_trajectory/cancel (actionlib_msgs/GoalID)**  
发布空的消息可以停止正在执行中的轨迹运动  
example:  
`
$ rostopic pub /cute_arm_controller/follow_joint_trajectory/cancel actionlib_msgs/GoalID -- {}
`  

------
### Published Topics:

* **joint_states (sensor_msgs/JointState)**  
各个关节的当前状态

* **tf (tf2_msgs/TFMessage)**  
反映各轴间坐标转换关系

------
### Services:
*注：以下Service在仿真环境下是没有的。*  

* **cute_go_home (std_srvs/SetBool)**  
让机械臂回到初始位姿  
example:  
`
$ rosservice call /cute_go_home "data: false"
`  
or  
`
$ rosservice call /cute_go_home "data: true"
`

* **cute_torque_enable (std_srvs/SetBool)**  
设置机械臂使能状态  
example:  
使能：  
`
$ rosservice call /cute_torque_enable "data: true" 
`  
去使能：  
`
$ rosservice call /cute_torque_enable "data: false" 
`
