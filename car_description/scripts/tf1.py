#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    x = 0
    y = 0
    theta = 0
    while(1):
        br.sendTransform((x, y, 0),
                        tf.transformations.quaternion_from_euler(0, 0,theta),
                        rospy.Time.now(),
                        "world", #parent
                        "car_link" )#child
        x += 1
        rospy.sleep(1)
    rospy.spin()
