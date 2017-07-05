#!/usr/bin/env python
# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Copyright (C) 2017 Maik Heufekes, 05/07/2017.
# License, GNU LGPL, free software, without any warranty.

# -*- coding: iso-8859-1 -*-
import sys
import cProfile, pstats
import time 
import rospy
import roslib; roslib.load_manifest("moveit_python")
import moveit_commander
import moveit_msgs.msg
import baxter_interface
import geometry_msgs.msg
import argparse
import struct
import copy
import rospkg
from moveit_python import PlanningSceneInterface, MoveGroupInterface
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import Range
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from math import pi, sqrt
from operator import itemgetter
from std_msgs.msg import String
from copy import deepcopy

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

rospy.init_node("pnp")
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
leftgripper.open()
rightgripper.open()

def load_gazebo_models(table_pose=Pose(position=Point(x=0.7, y=0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose1=Pose(position=Point(x=0.6725, y=0.0765, z=-0.135)),
                       block_pose2=Pose(position=Point(x=0.55, y=-0.2, z=-0.135)),
                       block_pose3=Pose(position=Point(x=0.7, y=-0.1, z=-0.135)),
                       block_pose4=Pose(position=Point(x=0.58, y=-0.03, z=-0.135)),
                       block_reference_frame="base"):
    # Get Models' Path.
    model_path = rospkg.RosPack().get_path('baxter_pnp_one_arm_cartesian_sim')+"/models/"
    # Load Table SDF.
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    # Load Block1 URDF.
    block_xml1 = ''
    with open (model_path + "block/model1.urdf", "r") as block_file:
        block_xml1=block_file.read().replace('\n', '')
    # Load Block2 URDF.
    block_xml2 = ''
    with open (model_path + "block/model2.urdf", "r") as block_file:
        block_xml2=block_file.read().replace('\n', '')
    # Load Block3 URDF.
    block_xml3 = ''
    with open (model_path + "block/model3.urdf", "r") as block_file:
        block_xml3=block_file.read().replace('\n', '')
    # Load Block4 URDF.
    block_xml4 = ''
    with open (model_path + "block/model4.urdf", "r") as block_file:
        block_xml4=block_file.read().replace('\n', '')
    # Spawn Table SDF.
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block1 URDF.
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block1", block_xml1, "/",
                               block_pose1, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    # Spawn Block2 URDF.
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block2", block_xml2, "/",
                               block_pose2, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    # Spawn Block3 URDF.
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block3", block_xml3, "/",
                               block_pose3, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
    # Spawn Block4 URDF.
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block4", block_xml4, "/",
                               block_pose4, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models.
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out.
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block1")
        resp_delete = delete_model("block2")
        resp_delete = delete_model("block3")
        resp_delete = delete_model("block4")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

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
    attempts=0
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
        while not((abs(z_pos-goal_z)< 0.01) and (abs(y_pos-goal_y)< 0.01) and (abs(x_pos-goal_x)< 0.01)):
            a=right_arm.get_current_pose()
	    x_pos= a.pose.position.x
	    y_pos= a.pose.position.y
	    z_pos= a.pose.position.z          
            time.sleep(0.5)
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

        while not((abs(z_pos-goal_z)< 0.01) and (abs(y_pos-goal_y)< 0.01) and (abs(x_pos-goal_x)< 0.01)):
            a=left_arm.get_current_pose()
	    x_pos= a.pose.position.x
	    y_pos= a.pose.position.y
	    z_pos= a.pose.position.z
            time.sleep(0.5)

            if(attempts>6):
                print("----->cartesian path failed!<-----")       
                left_arm.set_pose_target(goal)
                left_arm.plan()
                left_arm.go(wait=True)
            attempts +=1   

def picknplace():
    # Define positions.
    pos1 = {'left_e0': -1.69483279891317, 'left_e1':  1.8669726956453, 'left_s0': 0.472137005716569, 'left_s1': -0.38852045702393034, 'left_w0': -1.9770933862776057, 'left_w1': -1.5701993084642143, 'left_w2': -0.6339059781326424, 'right_e0': 1.7238109084167481, 'right_e1': 1.7169079948791506, 'right_s0': 0.36930587426147465, 'right_s1': -0.33249033539428713, 'right_w0': -1.2160632682067871, 'right_w1': 1.668587600115967, 'right_w2': -1.810097327636719}
    lpos1 = {'left_e0': -1.69483279891317, 'left_e1':  1.8669726956453, 'left_s0': 0.472137005716569, 'left_s1': -0.38852045702393034, 'left_w0': -1.9770933862776057, 'left_w1': -1.5701993084642143, 'left_w2': -0.6339059781326424}
    rpos1 = {'right_e0': 1.7238109084167481, 'right_e1': 1.7169079948791506, 'right_s0': 0.36930587426147465, 'right_s1': -0.33249033539428713, 'right_w0': -1.2160632682067871, 'right_w1': 1.668587600115967, 'right_w2': -1.810097327636719}

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
    placegoal.position.y = 0.2
    # Open the gripper 4 mm above the tip of the tower.
    placegoal.position.z = 0
    placegoal.orientation.x = 1.0
    placegoal.orientation.y = 0.0
    placegoal.orientation.z = 0.0
    placegoal.orientation.w = 0.0

    # Define variables.
    table_size_x = 1.0
    table_size_y = 0.929999123509
    table_size_z = 0.75580154342
    center_x = 0.6
    center_y = -0.0441882472378
    center_z = -0.52509922829
    # The distance from the zero point in Moveit to the ground is 0.903 m.
    # The value is not allways the same. (look in Readme)
    center_z_cube= -0.903+table_size_z+0.0275/2
    j=0
    k=0
    u = 0
    locs_x = []
    # Initialize a list for the objects and the stacked cubes.
    objlist = ['obj01', 'obj02', 'obj03', 'obj04', 'obj05', 'obj06', 'obj07', 'obj08', 'obj09', 'obj10', 'obj11']
    boxlist= ['box01', 'box02', 'box03', 'box04', 'box05', 'box06', 'box07', 'box08', 'box09', 'box10', 'box11']

    #rightarm.move_to_joint_positions(rpos1)
    right_arm.set_pose_target(rpos)
    right_arm.plan()
    right_arm.go(wait=True)

    both_arms.set_joint_value_target(pos1)
    both_arms.plan()
    both_arms.go(wait=True)

    # Load Gazebo Models via Spawning Services.
    load_gazebo_models()
    # Remove models from the scene on shutdown.
    rospy.on_shutdown(delete_gazebo_models)
    # Wait for the All Clear from emulator startup.
    rospy.wait_for_message("/robot/sim/started", Empty)
    # Clear planning scene.
    p.clear()
    # Add table as attached object.
    p.attachBox('table', table_size_x, table_size_y, table_size_z, center_x, center_y, center_z, 'base', touch_links=['pedestal'])
    p.waitForSync()
    rightgripper.open()
    # cProfile to measure the performance (time) of the task.
    pr = cProfile.Profile()
    pr.enable()
    # Loop to continue pick and place until all objects are cleared from table.
    while k<4:
        locs_x = [0.6725+0.02, 0.55+0.02, 0.7+0.02, 0.58+0.02]
        locs_y = [0.0765+0.02,-0.2+0.02, -0.1+0.02, -0.03+0.02]
        orien = [0, 0, 0, 0]
        size = [5, 5, 5, 5]

        # Filter objects list to remove multiple detected locations for same objects.
        ind_rmv = []
        for i in range(0,len(locs_x)):
            if (locs_y[i] > 0.24):
                ind_rmv.append(i)
                continue
            for j in range(i,len(locs_x)):
                if not (i == j):
                    if sqrt((locs_x[i] - locs_x[j])**2 + (locs_y[i] - locs_y[j])**2)<0.018:
                        ind_rmv.append(i)
        
        locs_x = del_meth(locs_x, ind_rmv)
        locs_y = del_meth(locs_y, ind_rmv)
        orien = del_meth(orien, ind_rmv) 
        size = del_meth(size, ind_rmv)

        # Do the task only if there are still objects on the table.
        while u<len(locs_x):
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
	    xn = locs_x[u]
	    yn = locs_y[u]	
            # -0.16 is the z position to grip the objects on the table.
	    zn = -0.143
	    thn = orien[u]
	    sz = size[u]
	    if thn > pi/4:
	        thn = -1*(thn%(pi/4))

	    # Add the detected objects into the planning scene.
	    for i in range(1,len(locs_x)):
	        p.addBox(objlist[i], 0.05, 0.05, 0.0275, locs_x[i], locs_y[i], center_z_cube)
	    # Add the stacked objects as collision objects into the planning scene to avoid moving against them.
	    for e in range(0, k):
	        p.attachBox(boxlist[e], 0.05, 0.05, 0.0275, placegoal.position.x, placegoal.position.y, center_z_cube+0.0275*(e-1), 'base', touch_links=['cubes'])    
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
            move('right',approach_pickgoal, pickgoal)
            time.sleep(1)
            rightgripper.close()
            time.sleep(0.5)
            # Move back to the approach pickgoal.
            move('right',approach_pickgoal)

            # Define the approach placegoal.
            # Increase the height of the tower every time by 2.75 cm.
            approached_placegoal=deepcopy(placegoal)
            approached_placegoal.position.z = -0.143+(k*0.0275)+0.08
            # Define the placegoal.
            placegoal.position.z = -0.143+(k*0.0275)
            move('right',approached_placegoal, placegoal)
	    rightgripper.open()
            time.sleep(1)
            # Move to the approach placegoal.
            move('right',approached_placegoal)
            k += 1
            u += 1
	    # Move right arm to start position.
            both_arms.set_joint_value_target(pos1)
            both_arms.plan()
            both_arms.go(wait=True)

    pr.disable()
    sortby = 'cumulative'
    ps=pstats.Stats(pr).sort_stats(sortby).print_stats(0.0)


    moveit_commander.roscpp_shutdown()
    # Exit MoveIt.
    moveit_commander.os._exit(0)
    time.sleep(10)


def main():
    picknplace()
    return 0

if __name__ == '__main__':
    sys.exit(main())
