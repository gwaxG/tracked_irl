#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, sys, time, gym, rospy
import numpy as np
import time, random
from std_msgs.msg import String, Float64
from gazebo_msgs.msg import LinkStates, LinkState
from std_srvs.srv import Empty, EmptyRequest
from gym_tracked.envs.base_control_env import BaseControl
from gym_tracked.envs.stability_assesment import Criteria
from gazebo_msgs.srv import GetWorldProperties
from gym_tracked.envs import rotations as rt
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

from collections import namedtuple

from gym import error, spaces, utils
from gym.utils import seeding
from gym import spaces 

# Box observation = spaces.Box( np.array([-1., -1., -1.]), np.array([2., 1., 2.]) )
# Discrete actions = spaces.Discrete()

class TrackedEnv(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self, **kwargs):
        # action_type = 'discrete_dqn', reward_type = 'classic', action_num = '2', nesm = '0', polygon = '0', env_type = 'step', discretize = 'True', log_address = ""
        # unwrapping dictionary
        action_type = kwargs['action_type']
        reward_type = kwargs['reward_type']
        action_num = kwargs['action_num']
        nesm = kwargs['nesm']
        polygon = kwargs['polygon']
        env_type = kwargs['env_type']
        self.communicator = None
        discretize = bool(kwargs['discretize'])
        self.log_address = kwargs['log_address']
        self.env_creator = kwargs['env_creator']
        # self.stability_criteria = Criteria()
        self.start = 0
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.link_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=10)
        self.reset_gazebo = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.world_description = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        self.reset_gazebo(EmptyRequest())
        self.cumulated_reward = 0
        self.on_stair = False
        self.on_stair_cnt = 0
        self.cumulated_stabiltiy = 0
        self.cumulated_projection = 0
        self.ep_num = 0
        self.timestep = 0
        self.init_time = int(time.time())
        self.logging = []
        ACTIONS_NUM = 11
        # Predefinition of env_descr
        self.env_descr = self.parse_env_description()
        # self.zones = self.get_zones(self.env_descr)
        self.reward_type = reward_type
        self.env_type = env_type
        self.robot_name = 'tracked_robot::dummy'
        self.step_counter = 0
        # x-y bounding box for robot position
        Bounds = namedtuple('Bounds', 'xmin ymin xmax ymax')
        # Max flipper rotation angle
        ANGLE = 0.087
        MA = 0.785
        ME = 5.0
        self.critical_distance = 2.62
        # Distance from the robot to the goal
        # The robot obtains positive reward when it is closer then min distance
        self.DISTANCE_ON_LANDING = 0.4336
        self.EARLY_STOPPING = 0.4336
        # self.minimal_distance_init = ((self.env_descr['n_steps']*self.env_descr['h_step'])**2 +
        #                               (self.env_descr['x_initial']+self.env_descr['tread']*(self.env_descr['n_steps']-1)+DISTANCE_ON_LANDING)**2)**0.5
        # self.minimal_distance = self.minimal_distance_init
        # self.bounds = Bounds(
        #     -0.3,
        #     -0.5,
        #     self.env_descr['tread']*self.env_descr['n_steps'] + EARLY_STOPPING + self.env_descr['x_initial'],
        #     0.5)
        self.noise = None
        if self.env_type == 'target':
            self.observation_space = spaces.Box( 
                np.array([0., -MA, -MA]), # self.bounds.xmin, self.bounds.ymin, 0.
                np.array([self.critical_distance, MA, MA]) # , self.bounds.xmax, self.bounds.ymax, 6.28
            )
        elif self.env_type == 'step':
            # Calculated like sin(45) * flipper_length + 0.5 * base_height
            # Hard-coded to the robot description
            MAX_LEVERAGE = 0.16 
            # At least two thirds of max leverage could be located below the edge 
            # It is equal to the robot height
            COEF = 2.0/3.0
            self.observation_space = spaces.Box( 
                np.array([0., -COEF*MAX_LEVERAGE, -MA, -MA]), # self.bounds.xmin, self.bounds.ymin, 0.
                np.array([self.env_descr['horizon'], MAX_LEVERAGE, MA, MA]) # , self.bounds.xmax, self.bounds.ymax, 6.28
            )
            self.sigma = float(kwargs['sigma'])
            sigma_x =  self.sigma * self.env_descr['horizon']
            sigma_y = self.sigma *  MAX_LEVERAGE
            self.noise = {
                'x': lambda: np.random.normal(0, sigma_x),
                'y': lambda: np.random.normal(0, sigma_y),
            }


        if self.noise == None:
            self.noise = lambda : 0.

        if action_type == 'discrete':
            # self.action_space = spaces.MultiDiscrete([3,3])
            self.action_space = spaces.MultiDiscrete([3, 3])
        elif action_type == 'box':
            self.action_space = spaces.Box( 
            np.array([-ANGLE, -ANGLE]), 
            np.array([ANGLE, ANGLE])
        )
        elif action_type == 'discrete_dqn':
            self.action_space = spaces.Discrete(ACTIONS_NUM**2)
        if action_type == 'discrete_dqn':
            self.zero_actions = 4
        else:   
            self.zero_actions = [0 for i in range(self.action_space.shape[0])]
        self.first_time = True

        # self.memory = [self.minimal_distance_init]
        self.memory_states = []
        self.initial_state = LinkStates()
        self.initial_state_init = False
        self.ready = False
        self.names = {
            'base': 'tracked_robot::dummy',
            # 'target': 'stairs::step5',
            'step': 'stairs::step_edge'
        }
        self.model_state = {
            'base_pose': Pose(),
            'target_pose': Pose(),
            'step_pose': Pose()
        }
        # A prameter that defines stability per episode
        self.stability_per_step = 0
        self.projection_per_step = 0         

        # discretize means that the continuous case is discretized and pseudo continuous
        self.base_control = BaseControl(action_type,  discretize=discretize, actions_num=ACTIONS_NUM)
        self.gazebo_counter = 0
        rospy.init_node('Learning_OpenAI', anonymous=True)
        # rospy.Subscriber('/tracked_robot/joint_states', JointState, process.hinge_callback, queue_size=10)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.gazebo_callback, queue_size=10)              

        robot = {
            'height': 0.1, 
            'length': 0.64, 
            'max_angle': 3.14/4, 
            'flipper': 0.16, 
            'base': 0.32
        }
        self.nesm = bool(int(nesm))
        self.projection = bool(int(polygon))

        prms = {'criteria': ['nesm', 'projection'], 'robot': robot}
        prms.update(self.env_descr)
        self.stability = Criteria(**prms)

    def debug(self, timestamp):
        N = 30
        se = '*'*N
        # start
        print(se)
        print('Debugging at {}'.format(timestamp))
        # stair params
        print("HL: \n", self.HL, "\n")
        # zones
        print("zones: \n", self.zones, "\n")
        # end line
        # cumulations
        print("Cumulated reward and normalized one: \n", round(self.cumulated_reward, 2),
              round(self.cumulated_reward/self.minimal_distance_init, 2), "\n")
        # state
        print("State: \n", self.state, "\n")
        print('Minimal distance', self.minimal_distance_init)
        # end
        print(se)


    def update_env_parameters(self, params):
        # Update all env parameters depending on stair size
        # It is supposed to be called every time when the stair is remade
        # Main goal is to adjust parameters used in reward calculation
        H = params['H']
        L = params['L']
        # I self.env_descr
        self.env_descr['h_step'] = H
        self.env_descr['tread'] = L
        # minimal_distance_init
        self.minimal_distance_init = ((self.env_descr['n_steps']*self.env_descr['h_step'])**2 +
                                      (self.env_descr['x_initial']+self.env_descr['tread']*(self.env_descr['n_steps']-1) \
                                       +self.DISTANCE_ON_LANDING - self.start)**2)**0.5
        self.noise['distance'] = lambda : np.random.normal(0, self.sigma * self.minimal_distance_init)
        # critical_distance for env type target
        self.critical_distance = self.env_descr['x_initial'] + \
                                 self.env_descr['n_steps']*self.env_descr['h_step'] + self.DISTANCE_ON_LANDING
        # Divide it into zones
        self.zones = self.get_zones(self.env_descr)
        # bounds
        Bounds = namedtuple('Bounds', 'xmin ymin xmax ymax')
        self.bounds = Bounds(
            -0.3,
            -0.5,
            self.env_descr['tread']*self.env_descr['n_steps'] + self.EARLY_STOPPING + self.env_descr['x_initial'],
            0.5)
        # Stability
        self.stability.update_criteria(**self.env_descr)

    def parse_env_description(self):
        path = os.path.dirname(os.path.realpath(__file__))
        path = os.path.join(path, "env_description.txt")
        with open(path) as f:
            lines = f.read().split("\n")
            filtered_lines = list(filter(lambda exp: "=" in exp, lines))
            d = {line.split("=")[0].strip(): float(line.split("=")[1].strip()) for line in filtered_lines}
        return d

    def get_zones(self, env_descr):
        zones = [ 
            [   
                env_descr['x_initial'], 
                env_descr['h_step'] 
            ]
        ]
        for i in range( int(env_descr["n_steps"]) - 1 ):
            zones.append([zones[-1][0] + env_descr['tread'], zones[-1][1] + env_descr['h_step']])
        zones.append([zones[-1][0] + env_descr['horizon'],zones[-1][1]])
        return zones

    def gazebo_callback(self, model_state):
        self.gazebo_counter += 1
        # Updates every 100 ms
        rate = 100
        if float(self.gazebo_counter) % rate == 0:
            self.update_model_state(model_state)
            self.gazebo_counter = 0

    def reset(self, test=None):
        # Move the robot to the start
        self.base_control.apply_actions(self.zero_actions)
        self.base_control.reset()
        # Create new stair
        if self.communicator == None:
            if test == None:
                self.HL = self.env_creator.create_random_stair()
            else:
                self.HL = self.env_creator.HL_test[test]
            self.env_creator.delete_model()
            self.env_creator.made_model(**self.HL)
            self.env_creator.spawn_model()
        else:
            self.HL = self.communicator
            self.env_creator.delete_model()
            self.env_creator.made_model(**self.HL)
            self.env_creator.spawn_model()
        ROBOT_SIZE = 0.64
        self.start = self.env_descr['x_initial'] - ROBOT_SIZE / 2
        self.update_env_parameters(self.HL)

        # Changed after random env creation
        # self.memory = [self.minimal_distance_init]
        self.memory = [0.0]

        self.unpause(EmptyRequest())
        self.step_counter = 0

        self.on_stair = False
        self.on_stair_cnt = 0

        self.stability_per_step = 0
        self.projection_per_step = 0
        self.minimal_distance = self.minimal_distance_init
        self.cumulated_reward = 0
        self.cumulated_stabiltiy = 0
        self.cumulated_projection = 0

        if self.first_time:
            self.first_time = False
        else:
            self.model_state['base_pose'] = Pose()
            self.model_state['base_pose'] = Pose()
            # Reset applied efforts and flipper positions (waits for 2 seconds after)
            self.base_control.reset()
            self.reset_gazebo(EmptyRequest())
            # self.memory = [self.minimal_distance_init]
            self.memory_states = []
            self.ready = False
            self.base_control.ready == False
            time.sleep(0.25)
        print('Waiting for updates')
        while True:
            if self.ready == True:
                self.ready = False
                # hard-coded // initiating movement forward
                self.base_control.apply_actions(self.zero_actions)
                return self.get_state()[0]

    def flipper_penalisation(self, state):
        punish = 0.
        delta = 0.2
        if len(self.memory_states) == 1:
            if abs(self.memory_states[0][1]) > delta:
                punish += -0.2
            if abs(self.memory_states[0][2]) > delta:
                punish += -0.2
        else:
            if abs(self.memory_states[-1][1]-self.memory_states[-2][1]) > delta:
                punish += -0.2
            if abs(self.memory_states[-1][2]-self.memory_states[-2][2]) > delta:
                punish += -0.2
        return punish      

    def deviation_penalisation(self):
        # calculate vector to the target
        # if self.ready: # +TAB
        r_p = self.model_state['base_pose'].position
        t_p = self.model_state['target_pose'].position
        vect = [t_p.x - r_p.x, t_p.y - r_p.y]
        tz_angle = round((vect[1])/(vect[0]**2 + vect[1]**2), 3)
        # calculate base orientation
        quat = self.model_state['base_pose'].orientation
        quat = [quat.w, quat.x, quat.y, quat.z]
        rz_angle = round(rt.quat2euler(quat)[2], 3)
        # absolute difference
        abs_diff = abs(rz_angle - tz_angle)

        if abs_diff > 0.2:
            return -abs_diff*0.05
        else:
            return 0.

#     def stability_assesment(self):
#         reward = 0.
#         reward += self.stability_criteria.esm()
#         reward += self.stability_criteria.com_projection()
#         reward += self.stability_criteria.support_polygon()
#         return reward

    def stability_estimation(self, rpose, state):
        rconf = {'front': state[1], 'rear': state[2]}
        return self.stability.calculate(rpose, rconf)

    def calculate_reward(self, state, absolute_distance):
        '''
        Reward is defined as the reaveled distance to the goal.
        Thus, maximum reward is obtained when the robot hits the goal.
        Any movement backward is punished.
        min reward: 0
        max reward: 10
        :param state as list:
        :return: reward
        '''
        # General penalisation term
        punish = 0.
        # Penalisation for deviation
        # Comment it if the robot moves only forward/backward


        if self.reward_type == 'safe':
            # Usually for regression problem with 4 dimensional action space
            # punish = self.deviation_penalisation()
            print('Safe penalty')
            punish += self.flipper_penalisation(state)

        self.stability_per_step, self.projection_per_step = self.stability_estimation(self.model_state['base_pose'], state)
        # print('DBG', round(self.stability_per_step, 2), round(self.projection_per_step, 2))
        if self.nesm:
            punish += -self.stability_per_step

        if self.projection: # to put into stability class
            OPTIMAL_PROJECTION_RATIO = 1.0
            proj_deviation = abs(self.projection_per_step-1.0)
            punish += -proj_deviation# proj_deviation if proj_deviation > 0.01 else 0.

        # Penalty for being stucked stuckPenalty
        #if len(self.memory) > 10:
        #    array = np.array(self.memory)
        #    variance = np.var(array[-10:])**0.5
        #    if variance < 0.01:
        #        punish += 0.0

        # if len(self.memory) == 0:
        #     # Memorize distance to the goal at the end of step
        #     self.memory.append(absolute_distance)
        # No reward for the first step, because we can not estimated travelled distance
        #     return 0. + punish
        # else:
        # distance

        # dist_prev = self.memory[-1]
        # dist_cur = absolute_distance

        # threshold_dist is calculated as hypotenuse
        # from the robot stucked position in fron of staircases and its final position
        if absolute_distance > 0 and absolute_distance <= self.minimal_distance_init:
            dist_max_prev = max(self.memory)
            r = (absolute_distance - max(self.memory))/(self.minimal_distance_init)
            self.memory.append(absolute_distance)
        else:
            r = 0

        if r > 0:
            return r + punish
        else:
            return 0. + punish

    def calculate_relative_position(self, model_state):
        '''
        This function returns relative distance to the next step along x and z axes.
        '''

        # Position of base 
        pb = model_state['base_pose']
        # Calculating zones for the given stair (a zone is one flat region)
        # Robot position is calculated using the end of the region (next step edge)
        
        # Determine robot location within a zone
        whereisrobot = {i: pb.position.x - zone[0]  for i, zone in enumerate(self.zones)}
        whereisrobot = dict(filter(lambda elem: elem[1] < 0, whereisrobot.items()))
        # if len(whereisrobot.items()) == 0:
        #     key = 0
        # else:
        key = None
        for item in whereisrobot.items():
            if item[1] == max(whereisrobot.values()):
                key = item[0]
                break
        if key == None:
            X = 0.
            Z = 0.05
        else:
            target_step = self.zones[key]
            # Forming return values for x and z axes
            X = target_step[0] - pb.position.x
            Z = pb.position.z - target_step[1]
            # print('DEBUG', X, Z)
        return [X, Z]

    def dist_to_goal(self, model_state, goal):
        pb = model_state['base_pose']
        pt = model_state['target_pose']
        half_step_size = 0.03
        if pb.position.x > self.start:
            dist = (pb.position.x - pt.position.x) ** 2
            dist += (pb.position.y - pt.position.y) ** 2
            dist += (pb.position.z - (pt.position.z + half_step_size)) ** 2
            dist = dist ** 0.5
        else:
            dist = 0
        # Not distance but the relative coordinate from a point to the base
        # dist = (pt.position.x - pb.position.x)/abs(pb.position.x - pt.position.x) * dist

        if goal == 'target':
            return [dist], dist
        elif goal == 'step': # TODO
            state = self.calculate_relative_position(model_state)

            noise_x = self.noise['x']()
            noise_y = self.noise['y']()

            state[0] += self.noise['x']()
            state[1] += self.noise['y']()

            # dist += self.noise['distance']()
            # Not distance but the relative coordinate from a point to the base
            # dist = (pt.position.x - pb.position.x)/abs(pb.position.x - pt.position.x) * dist

            return state, dist

    def get_model_state(self):
        state = []
        state.append(self.model_state['base_pose'].position.x)
        state.append(self.model_state['base_pose'].position.y)
        state.append(self.model_state['base_pose'].orientation.z)
        return state

    def get_state(self):
        while self.ready == False:
            time.sleep(0.05)
        if self.env_type == 'target':
            env_state, absolute_distance = self.dist_to_goal(self.model_state, 'target')
        elif self.env_type == 'step':
            env_state, absolute_distance = self.dist_to_goal(self.model_state, 'step')
        state_hinges = self.base_control.get_state()
        # model_state = self.get_model_state()
        state = env_state + state_hinges #  + model_state #  + dist_to_step + state_hinges # + model_state
        self.ready = False

        # Noising
        # for elem in state:

        return state, absolute_distance

    def update_model_state(self, msg):
        # Checking if stairs are spawned
        # Needed to prevent errors
        # if 'stairs' in self.world_description().model_names:
        self.ready = True
        if self.initial_state_init != True:
            self.initial_state = msg
            self.initial_state_init = True
        # print('DEBUG')
        # print(self.names, '\n', msg.name)
        if 'stairs::step1' in msg.name:
            for k, v in self.names.items():
                ind = msg.name.index(self.names[k])
                self.model_state[k+'_pose'] = msg.pose[ind]

    def is_done(self, model_state):
        robot_pos = model_state['base_pose'].position
        dist_to_goal = self.dist_to_goal(model_state, 'target')[0]
        # DBG 50 100
        window = 25
        reward_correction = 0.
        stucked_variance = .01
        max_ep = 100

        # If the robot is farther from th e goal then long_dist, then the episode ends.
        if robot_pos.x < self.bounds.xmin:
            print('Movement in a wrong direction, end of episode')
            reward_correction = 0. # -100.# -100.
            self.base_control.apply_actions(self.zero_actions)
            return True, reward_correction
        elif robot_pos.y < self.bounds.ymin or robot_pos.y > self.bounds.ymax:
            print('Movement in a wrong SIDE direction, end of episode')
            reward_correction = 0. # -2.# -100.
            self.base_control.apply_actions(self.zero_actions)
            return True, reward_correction
        elif self.step_counter > max_ep:
            print('Maximum number of steps per episode is exceeded')
            reward_correction = 0.
            return True, reward_correction
        elif robot_pos.x > self.bounds.xmax:
            print('At the finish line, end of episode')
            reward_correction = 1.
            return True, reward_correction
        elif len(self.memory) > window:
            array = np.array(self.memory)
            variance = np.var(array[-window:])**0.5
            if variance < stucked_variance:
                print('Stucked, EOE')
                reward_correction = 0
                return True, reward_correction
            else:
                return False, reward_correction
        else:
            return False, reward_correction

    def wait(self, t):
        timeout = time.time() + t
        while True:
            if time.time() > timeout:
                break



    def step(self, actions, test=False):

        '''
        :param actions:
        :return: state, reward, done, info
        '''
        self.step_counter += 1
        # Wait until the robot will be 5 cm from stair
        # while True:
        #     print('step')
        #     if self.dist_to_goal(self.model_state, 'target')[0] < self.critical_distance:
        #         break
        if test != False:
            self.timestep += 1
        if len(actions) == 1:
            actions = actions[0]
        # print('DEBUG', actions)
        if 'stairs' in self.world_description().model_names:
            self.base_control.apply_actions(actions)
        else:
            print('HERE')
            self.base_control.apply_actions(self.zero_actions)
        # We wait here 0.5 seconds  until all actions will be applied, it should depend of acceleration 
        self.wait(0.25)
        state, absolute_distance = self.get_state()

        # Noise test
        # print('TEST real states {}'.format(state))
        # for i in range(len(state)):
        #     state[i] = random.random() * state[i]

        # DBG, used for dbg purpose
        self.state = state

        if self.model_state['base_pose'].position.x > 0.6 and self.model_state['base_pose'].position.x < 2.2:
            self.on_stair = True
        else:
            self.on_stair = False

        self.memory_states.append(state)
        reward = self.calculate_reward(state, absolute_distance)
        # print("Reward {} Distance {}".format(reward, absolute_distance))
        # print('Obtained reward {}'.format(reward))
        done, reward_correction = self.is_done(self.model_state)

        if done and reward_correction == 1.:
            reward = reward_correction - self.cumulated_reward
            self.cumulated_reward = 1.
        else:
            self.cumulated_reward += reward
        # DBG
        print('Step {}; reward {} \r'.format(self.step_counter, round(reward, 2)), end='')
        # print('Step {}; reward {}'.format(self.step_counter, round(reward, 2)), end='')

        if self.on_stair:
            self.on_stair_cnt += 1
            self.cumulated_stabiltiy += self.stability_per_step
            self.cumulated_projection += self.projection_per_step
        if done and test != False:
            print('Done, reward: \n    {}'.format(self.cumulated_reward))
            if self.on_stair_cnt == 0:
                self.cumulated_stabiltiy = 0.
                self.cumulated_projection = 1.
                self.logging.append([self.timestep, self.cumulated_reward, self.cumulated_stabiltiy, self.cumulated_projection, self.on_stair_cnt])
            else:
                self.logging.append([self.timestep, self.cumulated_reward, self.cumulated_stabiltiy, self.cumulated_projection/self.on_stair_cnt, self.on_stair_cnt])


            np.array(self.logging).dump(self.log_address)
            self.ep_num += 1
            print('#### \nEpisode: {} \nClosest distance to the goal: {} \nreward: {} \n####' \
                  .format(self.ep_num, max(self.memory)/self.minimal_distance_init, \
                          self.cumulated_reward))
        # self.debug(self.step_counter)

        return state, reward, done, {
                                        'stability': self.stability_per_step,
                                        'projection': self.projection_per_step
                                    }

    def close(self):

        print('Closing Gazebo here')

    def render(self):
        print('Nothing to render')
        

