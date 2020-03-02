import rospy
import numpy as np
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import time
class SimpleMove:
    def __init__(self):
        rospy.init_node('simple_move')
        print('Simple move initialized')
        self.listener = tf.TransformListener()
        self.goal = Pose()
        self.broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber('/odom', Odometry, self.update_control_command, queue_size=1)
        rospy.Subscriber('/goal', Pose, self.update_goal, queue_size=1)
        self.vhat = 0.5
        self.TIME_STEP = 1.0
        self.last_timestamp = 0
        self.phat = 0.5
        self.robot_state = Pose()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_goal = rospy.Publisher('/goal', Pose, queue_size=1)
        self.vel_cmd = Twist()
        rospy.spin()

    def update_goal(self, msg):
        X = msg.pose.pose.position.x
        Y = msg.pose.pose.position.y
        Z = msg.pose.pose.position.z
        print('XYZ', X, Y, Z)
        self.goal.position.x = X
        self.goal.position.y = Y
        self.goal.position.z = Z
        self.broadcaster.sendTransform([X, Y, Z],
                                       [0,0,0,1],
                                       rospy.Time.now(),
                                       "map",
                                       "goal")
        print('Goal is updated')

    def broadcast_base_goal(self, state):
        (trans, rot) = self.listener.lookupTransform('/base_link', '/goal', rospy.Time(0))
        print(rot[2])
        return None
        euler = tf.transformations.euler_from_quaternion(rot)
        quaternion = tf.transformations.quaternion_from_euler(0, 0,  euler[2] - yaw)
        self.broadcaster.sendTransform([0.5,0,0],
                                       quaternion,
                                       rospy.Time.now(),
                                       "goal_direction",
                                       "base_link")

    def update_control_command(self, state):
        # self.broadcast_base_goal(state)
        dist = self.get_distance(state)
        self.robot_state = state
        if dist < 0.3 and time.time() - self.last_timestamp > 1:
            print('At the goal')
            self.vel_cmd.angular.z = 0
            self.vel_cmd.linear.x = 0
            self.publisher.publish(self.vel_cmd)
        elif dist >= 0.3 and time.time() - self.last_timestamp > 1:
            print('Distance', dist)
            theta = self.get_angle(state)
            thetap = 2*np.arctan(np.tan(theta/2))
            vp = self.vhat * (1 - abs(theta) / np.pi)
            self.vel_cmd.angular.z = 1.5 * thetap
            self.vel_cmd.linear.x = vp
            print('v theta', thetap * 5, vp)
            self.publisher.publish(self.vel_cmd)
            self.last_timestamp = time.time()


    def get_distance(self, state):
        '''
        :return: distance from the robot to the goal B
        '''
        x = self.goal.position.x - state.pose.pose.position.x
        y = self.goal.position.y - state.pose.pose.position.y
        z = self.goal.position.z - state.pose.pose.position.z
        print('xyz', self.goal.position.x, self.goal.position.y, self.goal.position.z)
        return np.sqrt(x**2 + y**2 + z**2)

    def get_angle(self, state):
        '''
        :return: theta angle which represents deviation of the robot heading angle to the goal
        '''
        (trans, rot) = self.listener.lookupTransform('/base_link', '/goal', rospy.Time(0))
        norm = (trans[0]**2+trans[1]**2)**0.5
        v = [trans[0]/norm, trans[1]/norm]
        theta = np.arcsin(v[1])
        return theta


if __name__ == '__main__':
    SimpleMove()