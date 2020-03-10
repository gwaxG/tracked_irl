#!/usr/bin/env python

import rospy
import tf
import time


'''
It is not implemented.
'''


class InitOdom:
    def __init__(self):
        rospy.init_node('init_odom_broadcaster')
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

    def run(self):
        RATE = 10.0 # Hz
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('/ar_marker_1', '/ar_marker_0', rospy.Time(0))
                self.br.sendTransform(trans, rot, rospy.Time.now(), "map", "ar_marker_1")
                time.sleep(1/RATE)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                time.sleep(1/RATE)
                continue


if __name__ == '__main__':
    InitOdom().run()
