#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Joy
import time
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyRequest
from std_srvs.srv import Empty, EmptyRequest
from tracked_irl.msg import BaseMotorCmd, FlipMotorCmd

class TeleopControl:
    def __init__(self):
        print('Base control instance created')
        self.sim = bool(rospy.get_param("sim"))
        self.pub_track_cmd = rospy.Publisher('/drrobot_jaguar_v6_basemotor_cmd', BaseMotorCmd, queue_size=1)
        self.pub_flip_cmd = rospy.Publisher('/drrobot_jaguar_v6_flipmotor_cmd', FlipMotorCmd, queue_size=1)
        self.pwm = 0.1
        self.msg_tracks = BaseMotorCmd()
        self.msg_flips = FlipMotorCmd()
        self.state_flips = {
            'leftFront': 0,
            'rightFront': 0,
            'leftRear': 0,
            'rightRear': 0
        }
        self.ready = False
        self.MAX_ANGLE = 1.1775
        self.ANGLE_STEP = 0.2
        self.MAX_EFFORT = 2.5
        self.STEP_EFFORT = 0.1
        self.effort_left = 0.
        self.regime = False
        self.allowed = True
        self.effort_right = 0.
        self.action_pub = rospy.Publisher('/action_published', Float64, queue_size=1)
        self.action_pub_msg = Float64()
        self.reset_gazebo = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.bars = {
            '/tracked_robot/left_front_bar_hinge_position_controller/command': Float64(),
            '/tracked_robot/left_rear_bar_hinge_position_controller/command': Float64(),
            '/tracked_robot/right_front_bar_hinge_position_controller/command': Float64(),
            '/tracked_robot/right_rear_bar_hinge_position_controller/command': Float64()
        }
        self.wheels = {
            '/tracked_robot/left_front_bar_wheel_1_hinge_effort_controller/command': Float64(),
            '/tracked_robot/left_front_bar_wheel_2_hinge_effort_controller/command': Float64(),
            '/tracked_robot/left_front_bar_wheel_3_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_front_bar_wheel_4_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_front_bar_wheel_5_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_rear_bar_wheel_1_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_rear_bar_wheel_2_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_rear_bar_wheel_3_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_rear_bar_wheel_4_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_rear_bar_wheel_5_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_wheel_1_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_wheel_2_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_wheel_3_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_wheel_4_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_wheel_5_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_wheel_6_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_wheel_7_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_wheel_8_hinge_effort_controller/command':Float64(),
            '/tracked_robot/left_wheel_9_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_front_bar_wheel_1_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_front_bar_wheel_2_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_front_bar_wheel_3_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_front_bar_wheel_4_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_front_bar_wheel_5_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_rear_bar_wheel_1_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_rear_bar_wheel_2_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_rear_bar_wheel_3_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_rear_bar_wheel_4_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_rear_bar_wheel_5_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_wheel_1_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_wheel_2_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_wheel_3_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_wheel_4_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_wheel_5_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_wheel_6_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_wheel_7_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_wheel_8_hinge_effort_controller/command':Float64(),
            '/tracked_robot/right_wheel_9_hinge_effort_controller/command':Float64()
        }
        # Dict of publishers
        self.publishers = {}
        # Common dict for flippers and tracks
        d = dict(self.wheels)
        d.update(self.bars)
        for k, v in d.items():
            self.publishers[k] = rospy.Publisher(k, Float64, queue_size=1)


    def wrap(self,actions):
        if len(actions) == 0:
            raise ValueError('Actions are not defined')
        res = {
            '/tracked_robot/left_front_bar_hinge_position_controller/command': -1*actions[0],
            '/tracked_robot/left_rear_bar_hinge_position_controller/command': actions[1],
            '/tracked_robot/right_front_bar_hinge_position_controller/command': actions[0],
            '/tracked_robot/right_rear_bar_hinge_position_controller/command': -1*actions[1],
            'left': actions[2],
            'right': actions[3]
        }
        return res

    def apply_actions(self, actions):
        '''
        :param actions: 'left_front_bar_hinge', 'left_rear_bar_hinge',  'right_front_bar_hinge',
                'right_rear_bar_hinge', 'left_wheel_1_hinge', 'right_wheel_1_hinge'
        :return: None
        '''

        # Transform list of actions to dict of actions
        actions = self.wrap(actions)
        step_angle = 0.2
        step_effort = 2.0
        # Publish
        for bar_k, bar_v in self.bars.items():
            self.bars[bar_k].data += step_angle * actions[bar_k]
            if abs(self.bars[bar_k].data) < self.MAX_ANGLE:
                self.publishers[bar_k].publish(self.bars[bar_k])
            else:
                self.bars[bar_k].data -= step_angle * actions[bar_k]

        for wheel_k, wheel_v in self.wheels.items():
            if 'left' in wheel_k:
                action = actions['left']
            elif 'right' in wheel_k:
                action = actions['right']
            self.wheels[wheel_k].data = action
            self.publishers[wheel_k].publish(self.wheels[wheel_k])
        # To delete	
        self.action_pub.publish(self.action_pub_msg)

    def calculate_commands(self, cmd):
        # Track efforts
        if self.regime == False:
            self.effort_left = cmd['y']*self.MAX_EFFORT - self.MAX_EFFORT*cmd['x']
            self.effort_right = cmd['y']*self.MAX_EFFORT + self.MAX_EFFORT*cmd['x']
        else:
            if self.allowed:
                self.effort_left = self.STEP_EFFORT # cmd['y_'] * self.STEP_EFFORT
                self.effort_right = self.STEP_EFFORT # cmd['y_'] * self.STEP_EFFORT
            else:
                self.effort_left = 0.
                self.effort_right = 0.
        front = 0
        rear = 0
        if cmd['front_down'] == 1:
            front = -1
        elif cmd['front_up'] == 1:
            front = 1
        elif cmd['rear_down'] == 1:
            rear = -1
        elif cmd['rear_up'] == 1:
            rear = 1
        self.apply_actions([front, rear, self.effort_left, self.effort_right])

    def joy_callback(self, msg):

        if abs(msg.buttons[4]) != 0:
            self.reset()
        else:
            d = {}
            d['x'] = msg.axes[2]
            d['y'] = msg.axes[3]
            d['front_down'] = msg.buttons[2]
            d['front_up'] = msg.buttons[3]
            d['rear_down'] = msg.buttons[1]
            d['rear_up'] = msg.buttons[0]
            self.calculate_commands(d)

    def reset(self):
        for k in self.bars.keys():
            self.bars[k].data = 0.
        for k in self.wheels.keys():
            self.wheels[k].data = 0.
        self.effort_left = 0.
        self.effort_right = 0.
        self.reset_gazebo(EmptyRequest())

    def autonomous_callback(self, vel):
        left = vel.linear.x - vel.angular.z
        right = vel.linear.x + vel.angular.z
        print('left {} right {}'.format(left, right))
        actions = [0, 0, left, right]
        if self.sim:
            self.apply_actions(actions)
        else:
            self.apply_actions_jaguar(actions)

    def apply_actions_jaguar(self, actions):
        '''
        remember that the left main track velocity command has to be inverted.
        Thus, to move forward it is necessary to set -300, 300 units for left and right tracks
        :param actions: list of actions [front_bars, rear_bars, left, right]
        :return:
        '''
        PWM = 300
        left = actions[2]
        right = actions[3]
        max_ = max(abs(left), abs(right))
        if max_ == 0:
            self.msg_tracks.leftCmd = 0.0
            self.msg_tracks.rightCmd = 0.0
        else:
            self.msg_tracks.leftCmd = -PWM * left / max_
            self.msg_tracks.rightCmd = PWM * right / max_
        print('Applying actions to the jaguar left: {}, right: {} \n'.format(
            self.msg_tracks.leftCmd,
            self.msg_tracks.rightCmd
            ))
        #decision = raw_input("Should I apply these actions (y/n)?")
        # if decision == 'y':
        self.pub_track_cmd.publish(self.msg_tracks)


if __name__ == '__main__':
    tele = TeleopControl()
    rospy.init_node('Joystick_control', anonymous=True)
    rospy.Subscriber('/joy', Joy, tele.joy_callback, queue_size=1)
    rospy.Subscriber('/cmd_vel', Twist, tele.autonomous_callback, queue_size=1)
    rospy.spin()

'''
rosrun tf static_transform_publisher 2 0.5 0 0 0 0 1 map go 100 
'''