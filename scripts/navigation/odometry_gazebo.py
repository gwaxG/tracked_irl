#! /usr/bin/env python
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
import rospy
import numpy as np

'''
This node calls the gazebo model state service, retrieve the current robot pose and publish it.
It is more or less useless since markers are used.
'''


def main():
    rospy.init_node('odom_pub')
    odom_pub = rospy.Publisher('/odom', Odometry)
    exact_pub = rospy.Publisher('/position', Odometry)
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    odom = Odometry()
    odom_precise = Odometry()
    header = Header()
    header.frame_id = '/odom'

    model = GetModelStateRequest()
    model.model_name = 'tracked_robot'

    r = rospy.Rate(2)

    while not rospy.is_shutdown():
        result = get_model_srv(model)
        result_precise = get_model_srv(model)
        # Add some noise
        # Noise should take into account previous velocities TO NOTICE

        odom_precise.pose.pose = result.pose
        odom_precise.twist.twist = result.twist
        exact_pub.publish(odom_precise)

        distance = (result.pose.position.x ** 2 + result.pose.position.y ** 2) ** 0.5
        noise = 0  # np.random.normal(0, min(0.1, round(distance/50, 2)), 1)
        #    result.pose.position.x += noise[0]
        #    result.pose.position.y += noise[0]

        odom.pose.pose = result.pose
        odom.twist.twist = result.twist

        header.stamp = rospy.Time.now()
        odom.header = header
        odom_precise.header = header

        odom_pub.publish(odom)

        r.sleep()


if __name__ == '__main__':
    main()
