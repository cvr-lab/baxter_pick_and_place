#!/usr/bin/env python

# Copyright (C) 2017 Maik Heufekes, 05/07/2017.
# License, GNU LGPL, free software, without any warranty.

import sys
import cProfile, pstats
import time 
import rospy
import roslib; roslib.load_manifest("moveit_python")
import baxter_interface
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt
from operator import itemgetter

def del_meth(somelist, rem):
    # Function to remove objects from the list.
    for i in rem:
        somelist[i]='!' 
    for i in range(0,somelist.count('!')):
        somelist.remove('!')
    return somelist

def picknplace():
    # Initialize the planning scene interface.
    p = PlanningSceneInterface("base")
    # Connect the arms to the move group.
    g = MoveGroupInterface("both_arms", "base")
    gr = MoveGroupInterface("right_arm", "base")
    gl = MoveGroupInterface("left_arm", "base")
    # Create baxter_interface limb instance.
    rightarm = baxter_interface.limb.Limb('right')
    # Create baxter_interface gripper instance.
    rightgripper = baxter_interface.Gripper('right')
    rightgripper.calibrate()
    rightgripper.open()

    # Define the joints for the positions.
    jts_both = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    jts_right = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    jts_left = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    pos1 = [-1.441426162661994, 0.8389151064712133, 0.14240920034028015, -0.14501001475655606, -1.7630090377446503, -1.5706376573674472, 0.09225918246029519,1.7238109084167481, 1.7169079948791506, 0.36930587426147465, -0.33249033539428713, -1.2160632682067871, 1.668587600115967, -1.810097327636719]
    rpos1 = [1.7238109084167481, 1.7169079948791506, 0.36930587426147465, -0.33249033539428713, -1.2160632682067871, 1.668587600115967, -1.810097327636719]
    rpos2 = [1.8342575250183106, 1.8668546167236328, -0.45674277907104494, -0.21667478604125978, -1.2712865765075685, 1.7472041154052735, -2.4582042097778323]
    placegoal = PoseStamped() 
    placegoal.header.frame_id = "base"
    placegoal.header.stamp = rospy.Time.now()
    placegoal.pose.position.x = 0.55
    placegoal.pose.position.y = 0.28
    # Open the gripper 4 mm above the tip of the tower.
    placegoal.pose.position.z = 0
    placegoal.pose.orientation.x = 1.0
    placegoal.pose.orientation.y = 0.0
    placegoal.pose.orientation.z = 0.0
    placegoal.pose.orientation.w = 0.0
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
    g.moveToJointPosition(jts_both, pos1,  max_velocity_scaling_factor = 1, plan_only=False)
    # cProfile to measure the performance (time) of the task.
    pr = cProfile.Profile()
    pr.enable()
    # Loop to continue pick and place until all objects are cleared from table.
    while locs_x or start:
        # Only for the start.
	if start:
            start = 0	
	
        time.sleep(1)
        # Receive the data from all objects from the topic "detected_objects".
        temp = rospy.wait_for_message("detected_objects", PoseArray) 
        locs = temp.poses 

        locs_x = []
        locs_y = []
        orien = []
        size = []

        # Adds the data from the objects.
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
	        #p.attachBox(boxlist[e], 0.05, 0.05, 0.0275, placegoal.pose.position.x, placegoal.pose.position.y, center_z_cube+0.0275*(e-1), 'base', touch_links=['cubes'])   
            if k>0:
	        p.attachBox(boxlist[0], 0.07, 0.07, 0.0275*k, placegoal.pose.position.x, placegoal.pose.position.y, center_z_cube, 'base', touch_links=['cubes']) 
	    p.waitForSync()
            # Initialize the pickgoal.
	    pickgoal = PoseStamped() 
	    pickgoal.header.frame_id = "base"
	    pickgoal.header.stamp = rospy.Time.now()
	    pickgoal.pose.position.x = xn
	    pickgoal.pose.position.y = yn
	    pickgoal.pose.position.z = zn		
	    pickgoal.pose.orientation.x = 1.0
	    pickgoal.pose.orientation.y = 0.0
	    pickgoal.pose.orientation.z = 0.0
	    pickgoal.pose.orientation.w = 0.0

            # Move to the approach pickgoal (5 cm to pickgoal).
	    pickgoal.pose.position.z = zn+0.05
	    # Orientate the gripper --> uses function from geometry.py (by Mike Ferguson) to 'rotate a pose' given rpy angles. 
	    pickgoal.pose = rotate_pose_msg_by_euler_angles(pickgoal.pose, 0.0, 0.0, thn)
	    gr.moveToPose(pickgoal, "right_gripper", max_velocity_scaling_factor = 1, plan_only=False)
	    pickgoal.pose.position.z = zn
            # Move to the pickgoal.
	    gr.moveToPose(pickgoal, "right_gripper", max_velocity_scaling_factor = 1, plan_only=False)
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
	    pickgoal.pose.position.z = zn+0.05
	    gr.moveToPose(pickgoal, "right_gripper", max_velocity_scaling_factor = 1, plan_only=False)

	    if pressure_ok and z_force>-4:
                # Define the approach placegoal.
                # Increase the height of the tower every time by 2.75 cm.
	        placegoal.pose.position.z = -0.155+(k*0.0275)+0.08
                # Move to the approach pickgoal.
	        gr.moveToPose(placegoal, "right_gripper", max_velocity_scaling_factor = 1, plan_only=False)
                # Define the placegoal.
	        placegoal.pose.position.z = -0.155+(k*0.0275)
		# Move to the placegoal and open the gripper 4 mm above the tip of the tower.
	        gr.moveToPose(placegoal, "right_gripper", max_velocity_scaling_factor = 0.3, plan_only=False)        
	        rightgripper.open()
                k += 1
		# Move to the approach placegoal.
		placegoal.pose.position.z = -0.155+(k*0.0275)+0.08
		gr.moveToPose(placegoal, "right_gripper", max_velocity_scaling_factor = 1, plan_only=False)

	    # Move right arm to start position.
	    gr.moveToJointPosition(jts_right, rpos1, max_velocity_scaling_factor = 1, plan_only=False)    
    
    pr.disable()
    sortby = 'cumulative'
    ps=pstats.Stats(pr).sort_stats(sortby).print_stats(0.0)
    p.clear()

if __name__=='__main__':
    try:
        rospy.init_node('pnp', anonymous=True)
        picknplace()
    except rospy.ROSInterruptException:
        pass
