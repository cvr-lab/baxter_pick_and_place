#!/usr/bin/env python
# -*- coding: iso-8859-1 -*-

# Copyright (C) 2017 Maik Heufekes, 05/07/2017.
# License, GNU LGPL, free software, without any warranty.

import sys
import cProfile, pstats
import time 
import rospy
import roslib; roslib.load_manifest("moveit_python")
import moveit_commander
import moveit_msgs.msg
import baxter_interface
import geometry_msgs.msg
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import Range

# Define initial parameters.
rospy.init_node('pnp', anonymous=True)
# Initialize the move_group API.
moveit_commander.roscpp_initialize(sys.argv)
# Connect the arms to the move group.
both_arms = moveit_commander.MoveGroupCommander('both_arms')
right_arm = moveit_commander.MoveGroupCommander('right_arm')
left_arm = moveit_commander.MoveGroupCommander('left_arm')
# Allow replanning to increase the odds of a solution.
right_arm.allow_replanning(True)
left_arm.allow_replanning(True)
# Set the arms reference frames.
right_arm.set_pose_reference_frame('base')
left_arm.set_pose_reference_frame('base')
# Create baxter_interface limb instance.
leftarm = baxter_interface.limb.Limb('left')
rightarm = baxter_interface.limb.Limb('right')
# Initialize the planning scene interface.
p = PlanningSceneInterface("base")
# Create baxter_interface gripper instance.
leftgripper = baxter_interface.Gripper('left')
rightgripper = baxter_interface.Gripper('right')
leftgripper.calibrate()
rightgripper.calibrate()
leftgripper.open()
rightgripper.open()

def set_current_position(arm, *arg):
    # Function to add the current position as the first point for a movement.
    if(arm=='left'):
        current_position=left_arm.get_current_pose()
    if(arm=='right'):
        current_position=right_arm.get_current_pose()

    current_pos = geometry_msgs.msg.Pose()
    current_pos.position.x = current_position.pose.position.x
    current_pos.position.y = current_position.pose.position.y
    current_pos.position.z = current_position.pose.position.z
    current_pos.orientation.x = current_position.pose.orientation.x
    current_pos.orientation.y = current_position.pose.orientation.y
    current_pos.orientation.z = current_position.pose.orientation.z
    current_pos.orientation.w = current_position.pose.orientation.w
    i=len(arg)
    if(i==1):
	waypoints=arg[0]
        waypoints.append(current_pos)
    return current_pos

def move(arm, *arg):
    # The cartesian path will be interpolated at a resolution of 0.1 cm
    # which is why the eef_step in cartesian translation is specify as 0.001. 
    # The jump threshold is specify as 0.0, effectively disabled.
    # This function is limited to 3 points but more can be added.
    fraction = 0
    attempts=0
    state=0
    waypoints = []
    set_current_position(arm, waypoints)
    # i is the number of waypoints.
    i=len(arg)
    waypoints.append(arg[0])
    # "goal" is the endposition of the movement, if there are more points then it will contain the last one.
    goal=arg[0]
    goal_x=goal.position.x
    goal_y=goal.position.y
    goal_z=goal.position.z
    if(i>1):
        goal=arg[1]
        goal_x=goal.position.x
        goal_y=goal.position.y
        goal_z=goal.position.z
        waypoints.append(arg[1])
    if(i>2):
        goal=arg[2]
        goal_x=goal.position.x
        goal_y=goal.position.y
        goal_z=goal.position.z
        waypoints.append(arg[2])

    if(arm=='right'):
        right_arm.set_start_state_to_current_state()
        # This function computes a cartesian path for the waypoints. It calculates points with a
        # maximum step size of 1 mm between the waypoints. It return the plan and the fraction
        # which says how good it followed the requested trajectory.
        # (example: fraction= 0.95.454545 -> followed 95.454545% of requested trajectory)
        (plan, fraction) = right_arm.compute_cartesian_path (waypoints, 0.001, 0.0, True)
        right_arm.execute(plan, wait=True) 
        # Read the position of the right arm to compare it with the goal.
        a=right_arm.get_current_pose()
	x_pos= a.pose.position.x
	y_pos= a.pose.position.y
	z_pos= a.pose.position.z
        # Waiting up to 3 seconds that the goal position is reached. (If it fail state=1)
	# It is also required to check that the movement is finished because it continues directly
        # after the command right_arm.execute() with the next code lines.
        while not((abs(z_pos-goal_z)< 0.01) and (abs(y_pos-goal_y)< 0.01) and (abs(x_pos-goal_x)< 0.01)):
            a=right_arm.get_current_pose()
	    x_pos= a.pose.position.x
	    y_pos= a.pose.position.y
	    z_pos= a.pose.position.z        
            time.sleep(0.5)
            if(attempts>6):
                print("----->cartesian path failed!<-----")
                state=1
            attempts +=1
        time.sleep(1)
        return state       

    if(arm=='left'):
        left_arm.set_start_state_to_current_state()
        (plan, fraction) = left_arm.compute_cartesian_path (waypoints, 0.001, 0.0, True)
        left_arm.execute(plan, wait=True)
        # Read the position of the left arm to compare it with the goal.
        a=left_arm.get_current_pose()
	x_pos= a.pose.position.x
	y_pos= a.pose.position.y
	z_pos= a.pose.position.z

        while not((abs(z_pos-goal_z)< 0.01) and (abs(y_pos-goal_y)< 0.01) and (abs(x_pos-goal_x)< 0.01)):
            a=left_arm.get_current_pose()
	    x_pos= a.pose.position.x
	    y_pos= a.pose.position.y
	    z_pos= a.pose.position.z
            time.sleep(0.5)
            if(attempts>6):
                print("----->cartesian path failed!<-----") 
                state=1      
            attempts +=1
        time.sleep(1)
        return state       

def measure_zero_point():
    # This function find the height from the ground to the zero point in MoveIt with the base frame. (in my case 0.903 m)
    # It is necessary to know the height of the table or the object which stands in front of the robot.
    # Add the real table size in z direction.
    table_size_z = 0
    # Define positions.
    pos1 = {'left_e0': -1.69483279891317, 'left_e1':  1.8669726956453, 'left_s0': 0.472137005716569, 'left_s1': -0.38852045702393034, 'left_w0': -1.9770933862776057, 'left_w1': -1.5701993084642143, 'left_w2': -0.6339059781326424, 'right_e0': 1.7238109084167481, 'right_e1': 1.7169079948791506, 'right_s0': 0.36930587426147465, 'right_s1': -0.33249033539428713, 'right_w0': -1.2160632682067871, 'right_w1': 1.668587600115967, 'right_w2': -1.810097327636719}
    lpos1 = {'left_e0': -1.69483279891317, 'left_e1':  1.8669726956453, 'left_s0': 0.472137005716569, 'left_s1': -0.38852045702393034, 'left_w0': -1.9770933862776057, 'left_w1': -1.5701993084642143, 'left_w2': -0.6339059781326424}
    rpos1 = {'right_e0': 1.7238109084167481, 'right_e1': 1.7169079948791506, 'right_s0': 0.36930587426147465, 'right_s1': -0.33249033539428713, 'right_w0': -1.2160632682067871, 'right_w1': 1.668587600115967, 'right_w2': -1.810097327636719}

    m_z_start = geometry_msgs.msg.Pose()
    m_z_start.position.x = 0.55
    m_z_start.position.y = 0
    m_z_start.position.z = 0.0
    m_z_start.orientation.x = 1.0
    m_z_start.orientation.y = 0.0
    m_z_start.orientation.z = 0.0
    m_z_start.orientation.w = 0.0

 
    both_arms.set_joint_value_target(pos1)
    both_arms.plan()
    both_arms.go(wait=True)
    # cProfile to measure the performance (time) of the task.
    pr = cProfile.Profile()
    pr.enable()
    # Start to measure the height of the table.
    state=move("right", m_z_start)
    right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range)
    # At first move with 10 cm steps and if the range is smaller then 25 cm with 1 cm steps.
    while(right_ir_sensor.range> 0.25):
        if not state:
            m_z_start.position.z -=0.1
        state=move("right", m_z_start)
        right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
        print "-----> The distance to table:",right_ir_sensor.range,"m"
    while(right_ir_sensor.range> 0.12):
        if not state:
            m_z_start.position.z -=0.01
        state=move("right", m_z_start)
        right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
        print "-----> The distance to table:",right_ir_sensor.range,"m"
    time.sleep(3)
    right_ir_sensor =rospy.wait_for_message("/robot/range/right_hand_range/state", Range) 
    distance=right_ir_sensor.range
    print "-----> The distance to table:", distance,"m"
    a=right_arm.get_current_pose()
    z_pos= a.pose.position.z
    # The z position of the tip of the gripper and the distance between
    # the tip and the table must be added.
    # (zpos is negative for table < 90 cm)
    # The value 0.099 is the distance from the ir sensor to the gripper tip.
    offset_zero_point= table_size_z-z_pos+(distance-0.099)
    print "-----> The distance from the ground to the zero point is:", offset_zero_point," m"
    both_arms.set_joint_value_target(pos1)
    both_arms.plan()
    both_arms.go(wait=True)
    time.sleep(100)
    moveit_commander.roscpp_shutdown()
    # Exit MoveIt.
    moveit_commander.os._exit(0)
if __name__=='__main__':
    try:
        rospy.init_node('pnp', anonymous=True)
        measure_zero_point()
    except rospy.ROSInterruptException:
        pass
