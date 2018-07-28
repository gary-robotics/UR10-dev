#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Alex Wang'
__author__ = 'Ye ZHENG'

import rospy
import numpy as np
import moveit_commander
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse
import geometry_msgs.msg
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters
import tf

import sys
import perception
from gqcnn.msg import BoundingBox
from gqcnn.srv import GQCNNGraspPlanner




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
        rospy.loginfo (self.arm.get_pose_reference_frame())     #base 
        rospy.loginfo (self.arm.get_planning_frame())     #world 
        

        # messgae filter for image topic, need carefully set./camera/depth_registered/sw_registered/image_rect,,/camera/rgb/image_rect_color
	rospy.loginfo ('wait_for_message')   ##############
	rospy.wait_for_message("/camera/rgb/image_raw", Image)  ###########
	rospy.wait_for_message("/camera/depth_registered/image", Image)###########
	rospy.loginfo ('start_callback')###########
        self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth_registered/image", Image)
        self.camera_info_topic = "/camera/rgb/camera_info"
        self.camera_info = rospy.wait_for_message(self.camera_info_topic,CameraInfo)
	rospy.loginfo (self.camera_info.header.frame_id)   ######

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
        #rospy.loginfo('callback')
        # depth image should be processed in advance 
        self.color_msg = color_msg
        self.depth_msg = depth_msg

    # Do not use tf to transform coordinate, or it will cause error with position and orientation  
    def call_gqcnn_srv(self):
        #try:
        rospy.sleep(1)
        # call gqcnn service
        rospy.loginfo('call gqcnn')
        rospy.wait_for_service('plan_gqcnn_grasp')
        plan_gqcnn_grasp = rospy.ServiceProxy('plan_gqcnn_grasp',GQCNNGraspPlanner)
        self.response = plan_gqcnn_grasp(self.color_msg,self.depth_msg,self.camera_info,self.bounding_box)
        #rospy.loginfo (self.response.grasp.pose)
        grasp_pose_camera_frame = self.response.grasp.pose
        grasp_pose_camera_frame_stamped = geometry_msgs.msg.PoseStamped()
        grasp_pose_camera_frame_stamped.pose = grasp_pose_camera_frame
        grasp_pose_camera_frame_stamped.header.frame_id = 'camera_rgb_optical_frame'   #############
        grasp_pose_camera_frame_stamped.header.stamp = rospy.Time(0)
        self.grasp_success_prob = self.response.grasp.grasp_success_prob    
        #rospy.loginfo (grasp_pose_camera_frame)  #camera
        self.grasp_pose_base_frame = self.listener.transformPose('base',grasp_pose_camera_frame_stamped)
        #rospy.loginfo (self.grasp_pose_base_frame.pose)
        #####################
        position_quat = self.camera_transform_to_base(self.response)
        self.grasp_pose_base_frame.pose.position.x = position_quat[0]
        self.grasp_pose_base_frame.pose.position.y = position_quat[1]
        self.grasp_pose_base_frame.pose.position.z = position_quat[2]
        self.grasp_pose_base_frame.pose.orientation.w = position_quat[3]
        self.grasp_pose_base_frame.pose.orientation.x = position_quat[4]
        self.grasp_pose_base_frame.pose.orientation.y = position_quat[5]
        self.grasp_pose_base_frame.pose.orientation.z = position_quat[6]
        rospy.loginfo (self.grasp_pose_base_frame.pose)
        #####################
        self.pre_grasp_pose_base_frame = self.grasp_pose_base_frame
        self.pre_grasp_pose_base_frame.pose.position.z += 0.4
        #rospy.loginfo (self.pre_grasp_pose_base_frame.pose)
                
        #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #rospy.loginfo ('look up transform failed')


    def Rot_to_Quat(self,rotation_matrix):
        r11 = rotation_matrix[0,0]
        r12 = rotation_matrix[0,1]
        r13 = rotation_matrix[0,2]
        r21 = rotation_matrix[1,0]
        r22 = rotation_matrix[1,1]
        r23 = rotation_matrix[1,2]
        r31 = rotation_matrix[2,0]
        r32 = rotation_matrix[2,1]
        r33 = rotation_matrix[2,2]
        q0 = 0.5*(1+r11+r22+r33)**0.5
        if q0 != 0:
            q1 = (r32 - r23)/(4*q0)
            q2 = (r13 - r31)/(4*q0)
            q3 = (r21 - r12)/(4*q0)
        else:
            q0 = 0
            q1 = 0.5*(1+r11-r22-r33)**0.5
            q2 = 0.5*(1+r22-r33-r11)**0.5
            q3 = 0.5*(1+r33-r11-r22)**0.5
            if q1 > q2 and q1 > q3:
                if r21 < 0:
                    q2 = -q2
                if r31 < 0 :
                    q3 = -q3
            elif q2 > q3:
                if r21 < 0 :
                    q1 = -q1
                if r32 < 0 :
                    q3 = -q3
            else :
                if r13 < 0:
                    q1 = -q1
                if r23 < 0:
                    q2 = -q2
        q = (q0,q1,q2,q3)
        return q


    def camera_transform_to_base(self,grasp_orientation):
        qx = grasp_orientation.grasp.pose.orientation.x
        qy = grasp_orientation.grasp.pose.orientation.y
        qz = grasp_orientation.grasp.pose.orientation.z
        qw = grasp_orientation.grasp.pose.orientation.w
        
        a1 = 1- 2*qz*qz- 2*qy*qy
        a2 = -2* qz*qw+2*qy*qx
        a3 = 2*qy*qw +2* qz*qx
        a4 = 2*qx*qy+ 2*qw*qz
        a5 = 1 - 2*qz*qz - 2*qx*qx
        a6 = 2*qz*qy- 2*qx*qw
        a7 = 2*qx*qz- 2*qw*qy
        a8 = 2*qy*qz + 2*qw*qx
        a9 = 1- 2*qy*qy- 2*qx*qx
        b1 = np.array([[a1,a2,a3,self.response.grasp.pose.position.x],
                    [a4,a5,a6,self.response.grasp.pose.position.y],
                    [a7,a8,a9,self.response.grasp.pose.position.z],
                    [0,0,0,1]])
        #b2 = np.array([[0.0505159, 0.998595 ,0.0160008,1.0724],
                    #[0.998593, -0.0502444, -0.0169399, 0.3202],
                    #[-0.0161121, 0.0168341, -0.999728, 0.9910],
                    #[0,0,0,1]])
	#b2 is the transformatin matrix of base-to-camera
        b2 = np.array([[-0.997932481939991,0.0227609013779336,0.0601057639295386,0.736057601275998],
          [0.0223614677411285,0.999723226802818,-0.00730989407734954,0.57023187242705],
          [-0.0602555080432645,-0.0059507276381616,-0.998165248138315,0.899734276240749],
          [0  , 0 ,  0  ,    1]])
        a = np.dot(b2,b1)
        quat = self.Rot_to_Quat(a)
        posit = (a[0,3],a[1,3],a[2,3])
        posit_quat = posit + quat
        return posit_quat



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
    rospy.sleep(10)   #wait for completing callback(self.cb)
    

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
          #move_plan.gripper.set_named_target('open')
          #move_plan.gripper.go(wait = True)
          #move_plan.gripper.set_named_target('open')
          #move_plan.gripper.go(wait = True)
          rospy.sleep(2)
          rospy.loginfo('gripper opened')

			# call gqcnn
          move_plan.call_gqcnn_srv()

          #move_plan.arm.set_pose_target(move_plan.pre_grasp_pose_base_frame.pose)
          #pre_plan = move_plan.arm.plan()
          #rospy.loginfo(bool(pre_plan.joint_trajectory.points))
          #rospy.loginfo(pre_plan)

          if move_plan.compute_ik_pose(move_plan.pre_grasp_pose_base_frame):
             move_plan.arm.set_pose_target(move_plan.pre_grasp_pose_base_frame) 
             move_plan.arm.go()
             move_plan.arm.clear_pose_targets()
             rospy.loginfo ('arrive at pre-grasp pose now !')
             rospy.sleep(1)     
             rospy.loginfo (move_plan.arm.get_current_pose())

             if move_plan.compute_ik_pose(move_plan.grasp_pose_base_frame):
                 move_plan.grasp_pose_base_frame.pose.position.z -= 0.22
                 move_plan.arm.set_pose_target(move_plan.grasp_pose_base_frame)        
                 move_plan.arm.go()
                 move_plan.arm.clear_pose_targets()
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
                 #move_plan.gripper.set_named_target('close')
                 #move_plan.gripper.go(wait = True)
                 #move_plan.gripper.set_named_target('close')
                 #move_plan.gripper.go(wait = True)
                 rospy.loginfo('gripper closed')
                 rospy.sleep(2)


					###################
                 move_plan.grasp_pose_base_frame.pose.position.z += 0.1
                 move_plan.arm.set_pose_target(move_plan.grasp_pose_base_frame)        
                 move_plan.arm.go()
                 move_plan.arm.clear_pose_targets()
					###################


                 move_plan.arm.set_named_target('place')
                 move_plan.arm.go()
                 rospy.loginfo('arrive at place pose now !')

                 move_plan.gripper.set_named_target('open')
                 move_plan.gripper.go()
                 #move_plan.gripper.set_named_target('open')
                 #move_plan.gripper.go(wait = True)
                 #move_plan.gripper.set_named_target('opecn')
                 #move_plan.gripper.go(wait = True)
                 rospy.loginfo('gripper opened')
                 rospy.sleep(2)
    rospy.spin()    

