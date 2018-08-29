#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = 'Alex Wang'

import sys
import numpy as np

import rospy
import tf
import geometry_msgs.msg
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge



import perception
from gqcnn.msg import BoundingBox
from gqcnn.srv import GQCNNGraspPlanner




class Grasp(object):
    def __init__(self):
        rospy.init_node("gqcnn_base_grasp",anonymous=True)
        
        # messgae filter for image topic, need carefully set./camera/depth_registered/sw_registered/image_rect,,/camera/rgb/image_rect_color
        rospy.wait_for_message("/camera/rgb/image_raw", Image)  ###########
        rospy.wait_for_message("/camera/depth/image", Image)###########
        self.image_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("/camera/depth/image", Image)
        self.camera_info_topic = "/camera/rgb/camera_info"
        self.camera_info = rospy.wait_for_message(self.camera_info_topic,CameraInfo)
        #rospy.loginfo (self.camera_info.header.frame_id)  

        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 1, 1)
        self.ts.registerCallback(self.cb)
        self.color_msg = Image()
        self.depth_msg = Image()
        
        # bounding box for objects, this need to be specified by the detection results
        self.bounding_box = BoundingBox()
        self.bounding_box.maxX = 420
        self.bounding_box.maxY = 250
        self.bounding_box.minX = 90
        self.bounding_box.minY = 40

        # operation related to OpenCV
        self.bridge = CvBridge()

        # transform listener
        self.listener = tf.TransformListener()



    def cb(self,color_msg,depth_msg):
        rospy.loginfo('callback')
        self.color_msg = color_msg
        self.depth_msg = depth_msg


    def call_gqcnn_srv(self):
        rospy.sleep(1)
        # call gqcnn service
        rospy.loginfo('call gqcnn')
        rospy.wait_for_service('plan_gqcnn_grasp')
        plan_gqcnn_grasp = rospy.ServiceProxy('plan_gqcnn_grasp',GQCNNGraspPlanner)
        self.response = plan_gqcnn_grasp(self.color_msg,self.depth_msg,self.camera_info,self.bounding_box)
        rospy.loginfo ('pose return by gqcnn')
        rospy.loginfo (self.response.grasp.pose)

    

if __name__=='__main__':
    grasp = Grasp()
    rospy.spin()


