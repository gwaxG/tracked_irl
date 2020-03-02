#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyRequest
from sensor_msgs.msg import JointState
import os 
import sys
import time
from itertools import product

class BaseControl:
    def __init__(self, action_type: str, action_dimension=2, discretize=True, actions_num=11) -> None:
        print('Base control instance created OpenAI')
        self.discretize = discretize
        self.ready = False
        self.anum = action_dimension
        # where x is actions
        self.ACTIONS_NUM = actions_num
        x = [ (i - int(self.ACTIONS_NUM*0.5))/float((self.ACTIONS_NUM - 1)*0.5) for i in range(self.ACTIONS_NUM)]
        self.possible_actions = sorted([p for p in product(x, repeat=2)])
        self.action_type = action_type
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # corresponds to 5 degrees
        self.MAX_STEP_ANGLE = 0.0872
        self.STEP_ANGLE = self.MAX_STEP_ANGLE / int(actions_num * 0.5)
        # corresponds to 45 degrees
        self.MAX_ANGLE = 0.785
        self.MAX_EFFORT = 5.0
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
            self.publishers[k] = rospy.Publisher(k, Float64, queue_size=10)

    def update_hinges(self, msg):
        self.ready = True
        for name in msg.name:
            if 'effort' in name:
                ind = msg.name.index(name)
                self.wheels[name].data = msg.effort[ind]
            elif 'position' in name:
                ind = msg.name.index(name)
                self.bars[name].data = msg.position[ind]

    def get_state(self):
        '''
        return hinge states in the next order:
                'left_front_bar_hinge', 
                'left_rear_bar_hinge',  
                'right_front_bar_hinge',
                'right_rear_bar_hinge', 
                'left_wheel_1_hinge', 
                'right_wheel_1_hinge'
        :return: list of floats with length 6
        '''
        state = []
        for key in self.bars.keys():
            if 'right_front' in key:
                state.insert(0,self.bars[key].data)
            if 'left_rear' in key:
                state.append(self.bars[key].data)
        # Uncomment in case of velocity control
        # state.append(self.wheels['/tracked_robot/left_wheel_1_hinge_effort_controller/command'].data)
        # state.append(self.wheels['/tracked_robot/right_wheel_1_hinge_effort_controller/command'].data)
        return state

    def wrap(self,actions):
        # Avoid in case of box actions
        # print("ACTIONS BEFORE", actions)
        if self.action_type == 'discrete_dqn':
                actions = self.possible_actions[actions]

        if self.action_type == 'discrete':
            actions = [ action - 1 for action in actions]

        if self.discretize and self.action_type == 'box':
            actions = [int((self.ACTIONS_NUM - 1)*0.5*action/self.MAX_STEP_ANGLE+0.5)*self.STEP_ANGLE for action in actions]


        res = {
            '/tracked_robot/left_front_bar_hinge_position_controller/command': -1*actions[0],
            '/tracked_robot/left_rear_bar_hinge_position_controller/command': actions[1],
            '/tracked_robot/right_front_bar_hinge_position_controller/command': actions[0],
            '/tracked_robot/right_rear_bar_hinge_position_controller/command': -1*actions[1],
        }



        if self.action_type == 'box' and self.anum == 2:
            res['left'] = 1
            res['right'] = 1

        if self.action_type == 'box' and self.anum == 3:
            res['left'] = actions[2]
            res['right'] = actions[2]
        if self.action_type == 'box' and self.anum == 4:
            res['left'] = actions[2]
            res['right'] = actions[3]
        return res

    def apply_actions(self, actions):
        '''
        :param actions: 'left_front_bar_hinge', 'left_rear_bar_hinge',  'right_front_bar_hinge',
                'right_rear_bar_hinge', 'left_wheel_1_hinge', 'right_wheel_1_hinge'
        :return: None
        '''
        self.pause(EmptyRequest())
        # Transform list of actions to dict of actions
        actions = self.wrap(actions)
        step_effort = 5.0
        # Publish
        # print('###################################################')
        for bar_k, bar_v in self.bars.items():
            # print('----')
            # print('Applying to {}, actions {}'.format(bar_k, actions[bar_k]))
            if self.action_type == 'discrete' or self.action_type == 'discrete_dqn':
                #In case of Discrete actions	
                if abs(self.bars[bar_k].data + self.STEP_ANGLE*actions[bar_k]) < self.MAX_ANGLE:
                    self.bars[bar_k].data += self.MAX_STEP_ANGLE*actions[bar_k] if self.action_type == 'discrete' else self.MAX_STEP_ANGLE*actions[bar_k]
                else:
                    self.bars[bar_k].data = self.MAX_ANGLE * (self.bars[bar_k].data)/abs(self.bars[bar_k].data)

            elif self.action_type == 'box':
                # self.bars[bar_k].data = actions[bar_k]
                # In case of Box actions; ATTENTION if discrete box actions, then pay attention to += 
                if abs(self.bars[bar_k].data + actions[bar_k]) < self.MAX_ANGLE:
                    self.bars[bar_k].data += actions[bar_k]
                else:
                    self.bars[bar_k].data = self.MAX_ANGLE * (self.bars[bar_k].data)/abs(self.bars[bar_k].data)
            self.publishers[bar_k].publish(self.bars[bar_k])
        for wheel_k, wheel_v in self.wheels.items():
            # Having action = 1 we always apply step_effort
            if 'left' in wheel_k and self.action_type == 'box':
                action = actions['left']
            elif 'right' in wheel_k and self.action_type == 'box':
                action = actions['right']
            else:
                # It forces the robot to move only forwar, and maybe useless
                action = 1.
            self.wheels[wheel_k].data = step_effort * action
            if abs(self.wheels[wheel_k].data) < self.MAX_EFFORT:
                self.publishers[wheel_k].publish(self.wheels[wheel_k])
            else:
                # We apply max effort in the direction of rotation sign(data) * max_effort
                self.publishers[wheel_k].publish(self.wheels[wheel_k].data / abs(self.wheels[wheel_k].data) * self.MAX_EFFORT)
        self.unpause(EmptyRequest())
        

    def reset(self):
        # Set position of bars and wheel efforts to zero
        for bar_k, bar_v in self.bars.items():
            self.bars[bar_k].data = 0.
            self.publishers[bar_k].publish(self.bars[bar_k])
        for wheel_k, wheel_v in self.wheels.items():
            self.wheels[wheel_k].data = 0.
            self.publishers[wheel_k].publish(self.wheels[wheel_k])
        # Wait for x seconds, flippers should move to the initial position
        print('Waiting for flippers')
