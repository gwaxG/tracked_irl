#!/usr/bin/env python

import rospy
import tf
import time
from ar_track_alvar_msgs.msg import AlvarMarker
from nav_msgs.msg import Odometry

'''
Now, there is now angular and linear velocity estimations.
It could be added along with using of Kalmman filtering.
'''

class AROdom:
    def __init__(self):
        rospy.init_node('ar_odometry')
        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.odom = Odometry()

    def run(self):
        RATE = 10.0 # Hz
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('/map', '/ar_marker_0', rospy.Time(0))
                self.odom.pose.pose.position.x = trans[0]
                self.odom.pose.pose.position.y = trans[1]
                self.odom.pose.pose.position.z = trans[2]
                self.odom.pose.pose.orientation.x = rot[0]
                self.odom.pose.pose.orientation.y = rot[1]
                self.odom.pose.pose.orientation.z = rot[2]
                self.odom.pose.pose.orientation.w = rot[3]
                # print(tf.transformations.euler_from_quaternion(rot))
                self.publisher.publish(self.odom)
                time.sleep(1/RATE)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                time.sleep(1/RATE)
                continue


if __name__ == '__main__':
    AROdom().run()
