import rospy
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import time
class SimpleMove:
    def __init__(self):
        '''
        The main feature of this init is a odometry subscriber initialization
        whose callback function publish velocity commands if the distance from the goal to the robot is high.
        '''
        rospy.init_node('simple_move')
        print('Simple move initialized')
        self.listener = tf.TransformListener()
        self.goal = None
        self.clearance = True
        self.broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/odom', Odometry, self.update_control_command, queue_size=1)
        self.TIME_STEP = 1.0
        self.last_timestamp = 0
        self.phat = 0.5
        self.robot_state = Pose()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel_cmd = Twist()
        rospy.spin()

    def update_control_command(self, state):
        '''
        This callback checks the distance from the robot to the goal and, then, publish appropriate commands.
        :param state: current robot state
        :return:
        '''
        K = 0.5
        VHAT = 0.2
        dist = self.get_distance(state)
        # print('Distance {}'.format(dist))
        self.robot_state = state
        self.clearance = False if dist < 0.1 else True
        if self.clearance == False:
            print('Time to stop')
            self.vel_cmd.angular.z = 0
            self.vel_cmd.linear.x = 0
            self.last_timestamp = time.time()
            self.publisher.publish(self.vel_cmd)
            # check for the goal change
        elif time.time() - self.last_timestamp > 1:
            # print('Distance', dist)
            theta = self.get_angle(state)
            thetap = 2*np.arctan(np.tan(theta/2))
            print("theta {}".format(theta))
            if abs(theta) < 0.5:
                vp = VHAT * (1 - abs(theta) / np.pi)
            else:
                vp = 0.
            self.vel_cmd.angular.z = K * thetap
            # print("angular velocity {}".format(self.vel_cmd.angular.z))
            self.vel_cmd.linear.x = vp
            self.publisher.publish(self.vel_cmd)
            self.last_timestamp = time.time()

    def get_distance(self, state):
        '''
        :param state: current robot state
        :return: distance from the robot to the goal
        '''
        (trans, rot) = self.listener.lookupTransform('/map', '/goal', rospy.Time(0))
        # print("Robot state {}".format(state.pose.pose.position))
        # print("Goal in the map frame {}".format(trans))
        self.goal = trans
        x = trans[0] - state.pose.pose.position.x
        y = trans[1] - state.pose.pose.position.y
        z = trans[2] - state.pose.pose.position.z
        return np.sqrt(x**2 + y**2 + z**2)

    def get_angle(self, state):
        '''
        :param state: current robot state
        :return: theta angle which represents deviation of the robot heading angle to the goal
        '''
        PI = 3.14

        (trans, rot) = self.listener.lookupTransform('/base_link', '/goal', rospy.Time(0))

        norm = (trans[0]**2+trans[1]**2)**0.5
        v = [trans[0]/norm, trans[1]/norm]

        theta = np.arcsin(v[1]) if trans[0] > 0 else PI - np.arcsin(v[1])


        return theta


if __name__ == '__main__':
    SimpleMove()
