#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Alex Wang'

import rospy
import copy
import moveit_commander
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse
import geometry_msgs.msg
from sensor_msgs.msg import Image, CameraInfo, JointState
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import tf

import sys
import numpy as np 
import cv2
import yaml
import threading

import perception
from autolab_core import RigidTransform, YamlConfig
from gqcnn import GQCNN
from gqcnn import CrossEntropyAntipodalGraspingPolicy, RgbdImageState
from gqcnn.msg import GQCNNGrasp, BoundingBox
from gqcnn.srv import GQCNNGraspPlanner
from gqcnn import Visualizer as vis




class MovePlan(object):
    def __init__(self,config):
        rospy.init_node("gqcnn_base_grasp",anonymous=True)
        
        # Moveit! setup
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.gripper = moveit_commander.MoveGroupCommander('gripper')
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_reference_frame('base')
        self.arm.set_planner_id('SBLkConfigDefault')
        self.arm.set_planning_time(10)
        self.arm.set_max_velocity_scaling_factor(0.04)
        self.arm.set_max_acceleration_scaling_factor(0.04)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_workspace([-2,-2,-2,2,2,2])
        self.gripper.set_goal_joint_tolerance(0.2)         

        # messgae filter for image topic
        self.image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth_registered/hw_registered/image_rect", Image)
        self.camera_info_topic = "/camera/rgb/camera_info"
        self.camera_info = rospy.wait_for_message(self.camera_info_topic,CameraInfo,timeout=3)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 1, 1)
        self.ts.registerCallback(self.cb)
        self.color_msg = Image()
        self.depth_msg = Image()
        
        # bounding box for objects 
        self.bounding_box = BoundingBox()
        self.bounding_box.maxX = 640
        self.bounding_box.maxY = 480
        self.bounding_box.minX = 0
        self.bounding_box.minY = 0
        self.bridge = CvBridge()

        # transform listener
        self.listener = tf.TransformListener()

        # compute_ik service
        self.ik_srv = rospy.ServiceProxy('/compute_ik',GetPositionIK)
        rospy.loginfo("Waiting for /compute_ik service...")
        self.ik_srv.wait_for_service()
        rospy.loginfo("Connected!")
        self.service_request = PositionIKRequest()
        self.service_request.group_name = 'arm'
        self.service_request.timeout = rospy.Duration(2)
        self.service_request.avoid_collisions = True

        # signal
        self.start = 0


    def cb(self,color_msg,depth_msg):
        rospy.loginfo('callback')
        # depth image should be processed in advance 
        self.color_msg = color_msg
        depth_msg = self.bridge.imgmsg_to_cv2(depth_msg,'passthrough')
        depth_msg = perception.DepthImage(depth_msg,config_file)
        depth_msg = depth_msg.inpaint()
        self.depth_msg = self.bridge.cv2_to_imgmsg(depth_msg.data)

        
    def call_gqcnn_srv(self):
        try:
            rospy.sleep(1)
            # call gqcnn service
            rospy.loginfo('call gqcnn')
            plan_gqcnn_grasp = rospy.ServiceProxy('plan_gqcnn_grasp',GQCNNGraspPlanner)
            response = plan_gqcnn_grasp(self.color_msg,self.depth_msg,self.camera_info,self.bounding_box)
            grasp_pose_camera_frame = response.grasp.pose
            grasp_pose_camera_frame_stamped = geometry_msgs.msg.PoseStamped()
            grasp_pose_camera_frame_stamped.pose = grasp_pose_camera_frame
            grasp_pose_camera_frame_stamped.header.frame_id = 'camera_link'
            grasp_pose_camera_frame_stamped.header.stamp = rospy.Time(0)
            self.grasp_success_prob = response.grasp.grasp_success_prob    
            self.grasp_pose_base_frame =  self.listener.transformPose('base',grasp_pose_camera_frame_stamped)
            #rospy.loginfo (self.grasp_pose_base_frame.pose)
            self.pre_grasp_pose_base_frame = self.grasp_pose_base_frame
            self.pre_grasp_pose_base_frame.pose.position.z += 0.4
            #rospy.loginfo (self.pre_grasp_pose_base_frame.pose)
                
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo ('look up transform failed')


    def compute_ik_pose(self, grasp_pose):
        self.service_request.robot_state = self.robot.get_current_state()
        self.service_request.pose_stamped = grasp_pose
        try:
            self.resp = self.ik_srv.call(self.service_request)
            if(self.resp.error_code):
                rospy.loginfo ('compute IK succeed')
                self.start_ik = 0
                return 1
            else:
                rospy.loginfo ('compute IK fail')
                rospy.loginfo (self.resp)
                self.start_ik = 0
                return 0
        except rospy.ServiceException, e:
            rospy.loginfo ("Service call failed: %s"%e)
            return 0
    


if __name__=='__main__':
    config_file=rospy.get_param("cfg_file","/home/ros/ur10_ws/src/gqcnn/cfg/ros_nodes/grasp_planner_node.yaml")
    move_plan = MovePlan(config_file)
    rospy.sleep(2)

    # loop for plan, move and grasp
    while not rospy.is_shutdown():
        move_plan.start = 1
        if move_plan.start == 1:
            move_plan.start =0
            rospy.loginfo ('in main')

            move_plan.arm.set_start_state_to_current_state()

            # hold on     
            move_plan.arm.set_named_target('hold')
            move_plan.arm.go()
            rospy.loginfo('arrive at hold pose now !')
            rospy.loginfo('wait for moving!')

            # make sure the gripper could open
            move_plan.gripper.set_named_target('open')
            move_plan.gripper.go(wait = True)
            move_plan.gripper.set_named_target('open')
            move_plan.gripper.go(wait = True)
            move_plan.gripper.set_named_target('open')
            move_plan.gripper.go(wait = True)
            rospy.sleep(2)
            rospy.loginfo('gripper opened')

            # call gqcnn
            move_plan.call_gqcnn_srv()

            if move_plan.compute_ik_pose(move_plan.pre_grasp_pose_base_frame):
                move_plan.arm.set_pose_target(move_plan.pre_grasp_pose_base_frame) 
                move_plan.arm.go()
                move_plan.arm.clear_pose_targets()
                rospy.loginfo ('arrive at pre-grasp pose now !')
                rospy.sleep(1)     

                if move_plan.compute_ik_pose(move_plan.grasp_pose_base_frame):
                    move_plan.grasp_pose_base_frame.pose.position.z -= 0.22
                    move_plan.arm.set_pose_target(move_plan.grasp_pose_base_frame)        
                    move_plan.arm.go()
                    rospy.loginfo ('arrve at grasp pose now !')
                    rospy.sleep(2)
                
                    ''' seems strange, need development
                    # Cartesian Paths
                    waypoints = []
                    waypoints.append(move_plan.arm.get_current_pose().pose)
                    wpose = geometry_msgs.msg.Pose()
                    wpose.orientation = waypoints[0].orientation
                    wpose.position.x = waypoints[0].position.x 
                    wpose.position.y = waypoints[0].position.y
                    wpose.position.z = waypoints[0].position.z 
                    waypoints.append(copy.deepcopy(wpose))
                    grasp_plan, fraction = move_plan.arm.compute_cartesian_path(waypoints, 0.01, 0.0,avoid_collisions = True)
                    grasp_plan = move_plan.arm.retime_trajectory(move_plan.robot.get_current_state(),grasp_plan,0.02)
                    move_plan.arm.execute(grasp_plan)
                    rospy.sleep(2)
                    rospy.loginfo ('arrve at grasp pose now !')
                    '''
                       
                    move_plan.gripper.set_named_target('close')
                    move_plan.gripper.go(wait = True)
                    move_plan.gripper.set_named_target('close')
                    move_plan.gripper.go(wait = True)
                    move_plan.gripper.set_named_target('close')
                    move_plan.gripper.go(wait = True)
                    rospy.loginfo('gripper closed')
                    rospy.sleep(2)


                    move_plan.arm.set_named_target('place')
                    move_plan.arm.go()
                    rospy.loginfo('arrive at place pose now !')

                    move_plan.gripper.set_named_target('open')
                    move_plan.gripper.go(wait = True)
                    move_plan.gripper.set_named_target('open')
                    move_plan.gripper.go(wait = True)
                    move_plan.gripper.set_named_target('open')
                    move_plan.gripper.go(wait = True)
                    rospy.loginfo('gripper opened')
                    rospy.sleep(2)
                                        
    rospy.spin()    

    
