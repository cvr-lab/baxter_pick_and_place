#!/usr/bin/env python

# Copyright (C) 2017 Maik Heufekes, 05/07/2017.
# License, GNU LGPL, free software, without any warranty.

import sys
import cProfile, pstats
import time 
import rospy
import roslib; roslib.load_manifest("moveit_python")
import sys
import moveit_commander
import moveit_msgs.msg
import baxter_interface
import geometry_msgs.msg
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt
from operator import itemgetter
from std_msgs.msg import String
from copy import deepcopy

# Define initial parameters.
rospy.init_node('pnp', anonymous=True)
# Initialize the move_group API.
moveit_commander.roscpp_initialize(sys.argv)
# Connect the arms to the move group.
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

def del_meth(somelist, rem):
    # Function to remove objects from the list.
    for i in rem:
        somelist[i]='!' 
    for i in range(0,somelist.count('!')):
        somelist.remove('!')
    return somelist

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
    # Moreover, if the move isn't precisly enough and it couldn't find a cartesian
    # path it will try to find a normal path. This function is limited to 3 points but more can 
    # be added.
    fraction = 0
    attempts= 0
    error= 0
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
        # Waiting up to 3 seconds that the goal position is reached or it will compute a normal path.
	# It is also required to check that the movement is finished because it continues directly
        # after the command right_arm.execute() with the next code lines.
        while not((abs(z_pos-goal_z)< 0.01) and (abs(y_pos-goal_y)< 0.01) and (abs(x_pos-goal_x)< 0.01) or error):
            a=right_arm.get_current_pose()
	    x_pos= a.pose.position.x
	    y_pos= a.pose.position.y
	    z_pos= a.pose.position.z          
            time.sleep(0.5)
            b=rightarm.endpoint_effort()
            z_= b['force']
	    z_force= z_.z
	    if z_force<-4:  
                error= 1  
            if(attempts>6):
                print("----->cartesian path failed!<-----")
                right_arm.set_pose_target(goal)
                right_arm.plan()
                # The right arm move to the goal position and it continues after it finished.
                right_arm.go(wait=True) 
            attempts +=1

    if(arm=='left'):
        left_arm.set_start_state_to_current_state()
        (plan, fraction) = left_arm.compute_cartesian_path (waypoints, 0.001, 0.0, True)
        left_arm.execute(plan, wait=True)
        # Read the position of the left arm to compare it with the goal.
        a=left_arm.get_current_pose()
	x_pos= a.pose.position.x
	y_pos= a.pose.position.y
	z_pos= a.pose.position.z

        while not((abs(z_pos-goal_z)< 0.01) and (abs(y_pos-goal_y)< 0.01) and (abs(x_pos-goal_x)< 0.01) or error):
            a=left_arm.get_current_pose()
	    x_pos= a.pose.position.x
	    y_pos= a.pose.position.y
	    z_pos= a.pose.position.z
            time.sleep(0.5)
            b=leftarm.endpoint_effort()
            z_= b['force']
	    z_force= z_.z
	    if z_force<-4:  
                error= 1  
            if(attempts>6):
                print("----->cartesian path failed!<-----")    
                left_arm.set_pose_target(goal)
                left_arm.plan()
                left_arm.go(wait=True)
            attempts +=1

def picknplace():
    # Define positions.
    rpos = geometry_msgs.msg.Pose()
    rpos.position.x = 0.555
    rpos.position.y = 0.0
    rpos.position.z = 0.206
    rpos.orientation.x = 1.0
    rpos.orientation.y = 0.0
    rpos.orientation.z = 0.0
    rpos.orientation.w = 0.0

    lpos = geometry_msgs.msg.Pose()
    lpos.position.x = 0.65
    lpos.position.y = 0.6
    lpos.position.z = 0.206
    lpos.orientation.x = 1.0
    lpos.orientation.y = 0.0
    lpos.orientation.z = 0.0
    lpos.orientation.w = 0.0

    placegoal = geometry_msgs.msg.Pose()
    placegoal.position.x = 0.55
    placegoal.position.y = 0.28
    placegoal.position.z = 0
    placegoal.orientation.x = 1.0
    placegoal.orientation.y = 0.0
    placegoal.orientation.z = 0.0
    placegoal.orientation.w = 0.0

    # Define variables.
    offset_zero_point=0.903
    table_size_x = 0.714655654394
    table_size_y = 1.05043717328
    table_size_z = 0.729766045265
    center_x = 0.457327827197
    center_y = 0.145765166941
    center_z = -0.538116977368
    # The distance from the zero point in Moveit to the ground is 0.903 m.
    # The value is not allways the same. (look in Readme)
    center_z_cube= -offset_zero_point+table_size_z+0.0275/2
    pressure_ok=0
    j=0
    k=0
    start=1
    locs_x = []
    # Initialize a list for the objects and the stacked cubes.
    objlist = ['obj01', 'obj02', 'obj03', 'obj04', 'obj05', 'obj06', 'obj07', 'obj08', 'obj09', 'obj10', 'obj11']
    boxlist= ['box01', 'box02', 'box03', 'box04', 'box05', 'box06', 'box07', 'box08', 'box09', 'box10', 'box11']
    # Clear planning scene.
    p.clear()
    # Add table as attached object.
    p.attachBox('table', table_size_x, table_size_y, table_size_z, center_x, center_y, center_z, 'base', touch_links=['pedestal'])
    p.waitForSync()
    # Move both arms to start state.
    right_arm.set_pose_target(rpos)
    left_arm.set_pose_target(lpos)
    right_arm.plan()
    left_arm.plan()
    right_arm.go(wait=True)
    left_arm.go(wait=True)
    # cProfile to measure the performance (time) of the task.
    pr = cProfile.Profile()
    pr.enable()
    # Loop to continue pick and place until all objects are cleared from table.
    while locs_x or start:
        # Only for the start.
	if start:
            start = 0		

        time.sleep(0.5)
        # Receive the data from all objects from the topic "detected_objects".
        temp = rospy.wait_for_message("detected_objects", PoseArray) 
        locs = temp.poses 

        locs_x = []
        locs_y = []
        orien = []
        size = []

        # Add the data from the objects.
        for i in range(len(locs)):
            locs_x.append(locs[i].position.x) 
            locs_y.append(locs[i].position.y) 
            orien.append(locs[i].position.z*pi/180)
            size.append(locs[i].orientation.x)

        # Filter objects list to remove multiple detected locations for same objects.
        ind_rmv = []
        for i in range(0,len(locs)):
            if (locs_y[i] > 0.24 or locs_x[i] > 0.75):
                ind_rmv.append(i)
                continue
            for j in range(i,len(locs)):
                if not (i == j):
                    if sqrt((locs_x[i] - locs_x[j])**2 + (locs_y[i] - locs_y[j])**2)<0.018:
                        ind_rmv.append(i)
        
        locs_x = del_meth(locs_x, ind_rmv)
        locs_y = del_meth(locs_y, ind_rmv)
        orien = del_meth(orien, ind_rmv) 
        size = del_meth(size, ind_rmv)

        # Do the task only if there are still objects on the table.
        if locs_x: 
            # Clear planning scene.
	    p.clear() 
            # Add table as attached object.
            p.attachBox('table', table_size_x, table_size_y, table_size_z, center_x, center_y, center_z, 'base', touch_links=['pedestal'])
            # Sort objects based on size (largest first to smallest last). This was done to enable stacking large cubes.
            ig0 = itemgetter(0)
            sorted_lists = zip(*sorted(zip(size,locs_x,locs_y,orien), reverse=True, key=ig0))
            locs_x = list(sorted_lists[1])
            locs_y = list(sorted_lists[2])
            orien = list(sorted_lists[3])
            size = list(sorted_lists[0])
	    # Initialize the data of the biggest object on the table.
	    xn = locs_x[0]
	    yn = locs_y[0]	
            # -0.16 is the z position to grip the objects on the table.	
	    zn = -0.16
	    thn = orien[0]
	    sz = size[0]
	    if thn > pi/4:
	        thn = -1*(thn%(pi/4))

	    # Add the detected objects into the planning scene.
	    #for i in range(1,len(locs_x)):
	        #p.addBox(objlist[i], 0.05, 0.05, 0.0275, locs_x[i], locs_y[i], center_z_cube)
	    # Add the stacked objects as collision objects into the planning scene to avoid moving against them.
	    #for e in range(0, k):
	        #p.attachBox(boxlist[e], 0.05, 0.05, 0.0275, placegoal.position.x, placegoal.position.y, center_z_cube+0.0275*(e-1), 'base', touch_links=['cubes'])    
            if k>0:
	        p.attachBox(boxlist[0], 0.07, 0.07, 0.0275*k, placegoal.position.x, placegoal.position.y, center_z_cube, 'base', touch_links=['cubes']) 
	    p.waitForSync()
            # Initialize the approach pickgoal (5 cm to pickgoal).
            approach_pickgoal = geometry_msgs.msg.Pose()
            approach_pickgoal.position.x = xn
            approach_pickgoal.position.y = yn
            approach_pickgoal.position.z = zn+0.05
	
            approach_pickgoal_dummy = PoseStamped() 
            approach_pickgoal_dummy.header.frame_id = "base"
            approach_pickgoal_dummy.header.stamp = rospy.Time.now()
            approach_pickgoal_dummy.pose.position.x = xn
            approach_pickgoal_dummy.pose.position.y = yn
            approach_pickgoal_dummy.pose.position.z = zn+0.05
            approach_pickgoal_dummy.pose.orientation.x = 1.0
            approach_pickgoal_dummy.pose.orientation.y = 0.0
            approach_pickgoal_dummy.pose.orientation.z = 0.0
            approach_pickgoal_dummy.pose.orientation.w = 0.0

	    # Orientate the gripper --> uses function from geometry.py (by Mike Ferguson) to 'rotate a pose' given rpy angles. 
            approach_pickgoal_dummy.pose = rotate_pose_msg_by_euler_angles(approach_pickgoal_dummy.pose, 0.0, 0.0, thn)
            approach_pickgoal.orientation.x = approach_pickgoal_dummy.pose.orientation.x
            approach_pickgoal.orientation.y = approach_pickgoal_dummy.pose.orientation.y
            approach_pickgoal.orientation.z = approach_pickgoal_dummy.pose.orientation.z
            approach_pickgoal.orientation.w = approach_pickgoal_dummy.pose.orientation.w
            # Move to the approach goal and the pickgoal.
            pickgoal=deepcopy(approach_pickgoal)
            pickgoal.position.z = zn 
            move("right", approach_pickgoal, pickgoal)
            time.sleep(0.5)
            # Read the force in z direction.
            b=rightarm.endpoint_effort()
            z_= b['force']
	    z_force= z_.z
            # Search again for objects, if the gripper isn't at the right position and presses on an object.
	    #print("----->force in z direction:", z_force)
	    if z_force>-4:
                rightgripper.close()
                attempts=0
	        pressure_ok=1
                # If the gripper hadn't enough pressure after 2 seconds it opens and search again for objects.
	        while(rightgripper.force()<25 and pressure_ok==1):   
		    time.sleep(0.04)
		    attempts+=1
		    if(attempts>50):
                        rightgripper.open()
                        pressure_ok=0
	                print("----->pressure is to low<-----")
            else:
                print("----->gripper presses on an object<-----")

            # Move back to the approach pickgoal.
            pickgoal.position.z = zn+0.05
            move("right", pickgoal)

	    if pressure_ok and z_force>-4:
                # Define the approach placegoal.
                # Increase the height of the tower every time by 2.75 cm.
                approached_placegoal=deepcopy(placegoal)
                approached_placegoal.position.z = -0.155+(k*0.0275)+0.08
                # Define the placegoal
                placegoal.position.z = -0.155+(k*0.0275)
		# Move to the placegoal and open the gripper 4 mm above the tip of the tower.
                move("right", approached_placegoal, placegoal)
	        rightgripper.open()
                while(rightgripper.force()>10):
		    time.sleep(0.01)
		# Move to the approach placegoal.
                move("right", approached_placegoal)
                k += 1

	    # Move right arm to start position.
	    right_arm.set_pose_target(rpos)
            right_arm.plan()
            right_arm.go(wait=True) 

    pr.disable()
    sortby = 'cumulative'
    ps=pstats.Stats(pr).sort_stats(sortby).print_stats(0.0)
    p.clear()
    moveit_commander.roscpp_shutdown()
    # Exit MoveIt.
    moveit_commander.os._exit(0)
if __name__=='__main__':
    try:
        rospy.init_node('pnp', anonymous=True)
        picknplace()
    except rospy.ROSInterruptException:
        pass
