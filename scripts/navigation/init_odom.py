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
        self.first_launch = True

    def run(self):
        RATE = 10.0 # Hz
        if self.first_launch:
            (trans, rot) = self.listener.lookupTransform("/ar_marker_1", "/usb_cam", rospy.Time(0))
            trans[2] = 0
            self.world_origin.append(trans)
            self.world_origin.append(rot)
            self.first_launch = False
        else:
            while not rospy.is_shutdown():
                # I. Publish marker_1 -> world transform;
                # usb -> world tree
                self.br.sendTransform(self.world_origin[0], self.world_origin[1], rospy.Time.now(),  "/ar_marker_1",
                                      "/world")


if __name__ == '__main__':
    InitOdom().run()
