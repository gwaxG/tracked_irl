#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import time


'''
This node has to connect 3 separated tf trees: base_link, usb_cam, world. 
'''


class InitOdom:
    def __init__(self):
        rospy.init_node('init_odom_broadcaster')
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.world_origin = []

    def run(self):
        RATE = 10.0 # Hz
        # A small trick used to make the listener waiting for the
        while True:
            try:
                (trans, rot) = self.listener.lookupTransform("ar_marker_1",
                                                             "usb_cam", rospy.Time(0))
                break
            except Exception as e:
                continue
        # trans[2] = 0
        self.world_origin.append(trans)
        self.world_origin.append(rot)
        while not rospy.is_shutdown():
            # I. Publish marker_1 -> world transform;
            # usb -> world tree
            self.br.sendTransform([0, 0, 0],
                                  [0, 0, 0,1],
                                  rospy.Time.now(),
                                  "/world",
                                  "/ar_marker_1"
                                  )

            self.br.sendTransform([-0.35, 0, -0.33],
                                  [0, 0, 0, 1],
                                  rospy.Time.now(),
                                  "/base_link",
                                  "/ar_marker_0"
                                  )

if __name__ == '__main__':
    InitOdom().run()
'''
rosrun tf static_transform_publisher 1.0 1.0 0.0 0.0 0.0 0.0 1.0 map goal 1000
'''

