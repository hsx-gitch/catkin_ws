#!/usr/bin/env python

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Pose
from arm_utils import scale_trajectory_speed

class MoveSpeedDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_speed_demo')
                
        # Initialize the move group for the right arm
        arm = MoveGroupCommander('arm')
        
        # Get the name of the end-effector link
        end_effector_link = arm.get_end_effector_link()
        
        # Allow replanning to increase the odds of a solution
        arm.allow_replanning(True)
        
        # Allow some leeway in position(meters) and orientation (radians)
        arm.set_goal_position_tolerance(0.02)
        arm.set_goal_orientation_tolerance(0.1)
        
        # Start the arm in the "resting" pose stored in the SRDF file
        arm.set_named_target('home')
        arm.go()
        
        # Move back to the resting position at roughly 1/4 speed
        arm.set_named_target('forward')
        arm.go()

        # Start the arm in the "resting" pose stored in the SRDF file
        arm.set_named_target('home')
        arm.go()

        # Get back the planned trajectory
        arm.set_named_target('forward')
        traj = arm.plan()
        
        # Scale the trajectory speed by a factor of 0.25
        new_traj = scale_trajectory_speed(traj, 0.25)

        # Execute the new trajectory     
        arm.execute(new_traj)
        rospy.sleep(1)

        # Exit MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveSpeedDemo()

    
    
