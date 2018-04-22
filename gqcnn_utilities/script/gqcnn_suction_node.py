#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import copy
import moveit_commander
import sys

    while not rospy.is_shutdown():
        waypoints = []
        print ('in main')
        move_plan.gripper.set_named_target('open')
        move_plan.gripper.go(wait = True)
        rospy.sleep(2)
        move_plan.call_gqcnn_srv()
        move_plan.arm.set_pose_target(move_plan.pre_grasp_pose_base_frame)        
        pre_grasp_plan = move_plan.arm.plan()
        move_plan.arm.execute(pre_grasp_plan)
        rospy.sleep(2)
        # Cartesian Paths
        wpose = geometry_msgs.msg.Pose()
        wpose.orientation.w = 1.0
        wpose.position.z -= 0.24
        waypoints.append(copy.deepcopy(wpose))
        grasp_plan, fraction = move_plan.arm.compute_cartesian_path(waypoints, 0.01, 0.0,avoid_collisions = True)
        plan = moveit_
        move_plan.arm.execute(grasp_plan)
        rospy.sleep(2)
        move_plan.gripper.set_named_target('close')
        move_plan.gripper.go(wait = True)
        rospy.sleep(2)
        