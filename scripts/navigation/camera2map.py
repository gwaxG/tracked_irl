#!/usr/bin/env python

import rospy
import tf
import time


'''
It is not implemented.
'''


class Cam2Map:
    def __init__(self):
        rospy.init_node('camera2map_broadcaster')
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

    def run(self):
        RATE = 10.0 # Hz
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('/camera_link', '/ar_marker_1', rospy.Time(0))
                (trans_, rot_) = self.listener.lookupTransform('/base_link', '/ar_marker_0', rospy.Time(0))
                print('cam m1', trans)
                print('base m0', trans_)
                # self.br.sendTransform(trans, rot, rospy.Time.now(), "stair", "map")
                time.sleep(1/RATE)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                time.sleep(1/RATE)
                continue


if __name__ == '__main__':
    Cam2Map().run()
