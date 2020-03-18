#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import time

class ARStair:
    def __init__(self):
        rospy.init_node('ar_stair_location')
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.marker_size = 0.22
        self.stair_shape = {'length': 0.8, 'height': 0.1, 'width': 1.2}

    def run(self):
        RATE = 10.0 # Hz
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('/map', '/ar_marker_1', rospy.Time(0))
                trans[0] += - self.stair_shape['length'] / 2
                trans[1] += - self.stair_shape['width'] / 2 - self.marker_size / 2
                trans[2]+= self.stair_shape['height']
                self.br.sendTransform(trans, rot, rospy.Time.now(), "stair", "map")
                time.sleep(1/RATE)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                time.sleep(1/RATE)
                continue


if __name__ == '__main__':
    ARStair().run()
