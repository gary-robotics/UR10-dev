#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters

import numpy as np 
import cv2
import yaml

import perception
from autolab_core import RigidTransform, YamlConfig
from gqcnn import GQCNN
from gqcnn import CrossEntropyAntipodalGraspingPolicy, RgbdImageState
from gqcnn.msg import GQCNNGrasp, BoundingBox
from gqcnn.srv import GQCNNGraspPlanner
from gqcnn import Visualizer as vis


class GQCNN_Pose(object):
    def __init__(self,config):
        self.image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth_registered/sw_registered/image_rect", Image)
        self.camera_info_topic = "/camera/rgb/camera_info"
        self.camera_info = rospy.wait_for_message(self.camera_info_topic,CameraInfo,timeout=3)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 1, 1)
        self.ts.registerCallback(self.cb)
        # bounding box for objects 
        self.bounding_box = BoundingBox()
        self.bounding_box.maxX = 640
        self.bounding_box.maxY = 480
        self.bounding_box.minX = 0
        self.bounding_box.minY = 0
        self.bridge = CvBridge()

    def cb(self,color_msg,depth_msg):
        # depth image should be processed in advance 
        self.color_msg = color_msg
        depth_msg = self.bridge.imgmsg_to_cv2(depth_msg,"passthrough")
        depth_msg = perception.DepthImage(depth_msg,config_file)
        depth_msg = depth_msg.inpaint()
        self.depth_msg = self.bridge.cv2_to_imgmsg(depth_msg.data)

        '''
        depth_msg = np.array(depth_msg, dtype=np.float32) / 1000.0
        depth_msg[np.isnan(depth_msg)] = 0
        cv2.normalize(depth_msg, depth_msg, 0, 1, cv2.NORM_MINMAX)
        depth_msg = np.expand_dims(depth_msg,2)    
        '''

        try:
            plan_gqcnn_grasp = rospy.ServiceProxy('plan_gqcnn_grasp',GQCNNGraspPlanner)
            response = plan_gqcnn_grasp(self.color_msg,self.depth_msg,self.camera_info,self.bounding_box)
            print(response)    
            while True:
                print(response)     

        except rospy.ServiceException, e:
            print("Service call failed:%s"%e)
        

if __name__=='__main__':
    rospy.init_node("gqcnn_node")
    config_file=rospy.get_param("cfg_file","/home/ros/ur10_ws/src/gqcnn/cfg/ros_nodes/grasp_planner_node.yaml")
    gqcnn_pose = GQCNN_Pose(config_file)
    rospy.spin()
    
