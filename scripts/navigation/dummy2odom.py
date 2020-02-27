#!/usr/bin/env python

import roslib
import rospy
import tf
import sys
import multiprocessing as mp
from nav_msgs.msg import Odometry
path = '/'.join(__file__.split('/')[:-2])
sys.path.append(path)

class Dummy2Odom:
    def __init__(self):
        rospy.init_node("dummy2odom")
        self.br = tf.TransformBroadcaster()
        # Broadcast odometry data
        rospy.Subscriber('/odom', Odometry, self.handle_pose)
        rospy.spin()

    def handle_pose(self, msg):
        print('Broadcasting dummy to odom')
        self.br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                         (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                          msg.pose.pose.orientation.w),
                         rospy.Time.now(),
                         "dummy",
                         "odom")


if __name__ == '__main__':
    Dummy2Odom()
