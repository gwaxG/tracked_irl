#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import gym
import rospy
import argparse
import numpy as np
from stable_baselines.her import GoalSelectionStrategy, HERGoalEnvWrapper
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.sac.policies import MlpPolicy as SACMlpPolicy
from stable_baselines.deepq.policies import MlpPolicy as DQNPolicy
from stable_baselines import DQN
from stable_baselines import PPO2, TD3, SAC
from stable_baselines.td3.policies import MlpPolicy as TD3MlpPolicy
from stable_baselines.common.policies import MlpPolicy as PPO2MlpPolicy
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
# from tracked_env.tracked_env import TrackedEnv
import gym_tracked, gym
# from tracked_env_linear.tracked_env import TrackedEnv
from gazebo_msgs.msg import LinkStates

from gym import envs
import tensorflow as tf
from stable_baselines.common.vec_env import SubprocVecEnv

class ExperimentBuilder:
    pass

class LearningOpenAI:
    def __init__(self, alg, reward_type, 
            action_type, action_num, comment, nesm, polygon, discretize=True):
        # 3 sets means that there will be tested lwo, medium and high parameter values
        # env = TrackedEnv(action_type, reward_type, action_num, nesm, polygon, discretize=True) 
        env = gym.make("Tracked-v0", **{'action_type': action_type, 'reward_type': reward_type, 'action_num' : action_num, 'nesm': nesm, 'polygon': polygon, 'env_type': 'step', 'discretize': True})
        self.env = DummyVecEnv([lambda: env])
        self.stability_counter = 0
        self.projection_counter = 0
        self.cur_ep = 0
        self.step = 0
        self.set_callback = True
        if alg != 'DQN':
            n_actions = env.action_space.shape[-1]
            action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
            ou_action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))
        self.name = alg+'_default_'+reward_type+ \
            '_'+action_type+'_'+action_num+'_actions_'+comment # +'_nesm_'+nesm+'_polygon_'+polygon

        if alg == 'PPO2':
            self.model = PPO2(PPO2MlpPolicy, self.env, n_steps=128, verbose=1, 
                tensorboard_log=self.get_path('logs'))

        if alg == 'SAC':
            self.model = SAC(SACMlpPolicy, self.env, verbose=1, action_noise=ou_action_noise,
                tensorboard_log=self.get_path('logs'))

        if alg == 'DQN':
            self.model = DQN(DQNPolicy, env, verbose=1, tensorboard_log=self.get_path('logs'))
            self.set_callback = False

        if alg == 'TD3':
            
            # action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), 
            #     sigma=float(0.5) * np.ones(n_actions))
            self.model = TD3(TD3MlpPolicy, self.env, verbose=1, random_exploration=0.1, # action_noise=action_noise, 
                tensorboard_log=self.get_path('logs'))
        self.model.is_tb_set = False

    def get_path(self, identificator, name=None):
        path = os.path.dirname(os.path.realpath(__file__))
        print('DEBUG', self.name)
        if name == None:
            path = os.path.join(path, identificator, self.name)
        else:
            path = os.path.join(path, identificator, name)
        # print('Returned path', path)
        return path

    def train_model(self):
        for i in range(1):
            if i > 0:
                self.model.load(self.get_path('learning_results'))                
            if self.set_callback:
                self.model.learn(total_timesteps=10000, callback=self.callback)
            else:   
                self.model.learn(total_timesteps=10000)

            self.model.save(self.get_path('learning_results'))

    def callback(self, locals_, globals_):
        self_ = locals_['self']
        # get_attr returns a list, stabiltiy is on the first place [0]
        env_ep = self_.env.get_attr('ep_num')[0]
        self.stability_counter += self_.env.get_attr('stability_per_step')[0]
        self.projection_counter += self_.env.get_attr('projection_per_step')[0]
        self.step += 1
        if self.cur_ep != env_ep:
            self.cur_ep = env_ep
            print('Episode {} stabiltiy {}'.format(self.cur_ep-1, self.stability_counter))
            summary = tf.Summary(value=[tf.Summary.Value(tag='instability_per_episode', simple_value=self.stability_counter/self.step)])
            locals_['writer'].add_summary(summary, self_.num_timesteps)
            summary = tf.Summary(value=[tf.Summary.Value(tag='projection_per_episode', simple_value=self.projection_counter/self.step)])
            locals_['writer'].add_summary(summary, self_.num_timesteps)
            self.projection_counter = 0
            self.stability_counter = 0
            self.step = 0
        return True

    @staticmethod
    def debug():
        env = TrackedEnv('box', 'classic', '2', 'True', 'True')
        while True:
            print(env.stability_estimation())

    def perform_demonstration(self,name):
        print('Performing demonstration, /{} is loaded'.format(name))
        self.model.load(self.get_path('learning_results', name))
        obs = self.env.reset()
        while True:
            action, _states = self.model.predict(obs)
            obs, rewards, dones, info = self.env.step(action)
        self.env.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('--mode', default='train', 
        help='Working mode either train or demonstration')

    parser.add_argument('--alg', default='TD3', 
        help='Define algorithm, available PPO2 and TD3')
    
    parser.add_argument('--rtype', default='classic', 
        help='Reward function type: classic, safe, stable')
    
    parser.add_argument('--atype', default='box', 
        help='Type of actions either discrete or box (continuous)')
    
    parser.add_argument('--anum', default='2', 
        help='Reward function type')

    parser.add_argument('--comment', default='', 
        help='Comment which will be added to the log name')

    parser.add_argument('--nesm', default='0', 
        help='Add NESM stability margin to reward function')

    parser.add_argument('--polygon', default='0', 
        help='Add support polygon estimation')

    args = parser.parse_args()
    
    if args.mode == 'train':

        # learning = LearningOpenAI(args.alg, args.rtype, args.atype, 
        #     args.anum, args.comment, args.nesm, args.polygon)
        # learning.train_model()
        for i in range(7):
            learning = LearningOpenAI('DQN', 'classic', 
                'discrete_dqn', '2', 'DQN_EXP', '0', '0', discretize=True)
            learning.train_model()



        # learning = LearningOpenAI('DQN', 'classic', 
        #     'discrete_dqn', '2', 'dqn_test', '0', '0')
        # learning.train_model()

        # learning = LearningOpenAI('TD3', 'classic', 
        #     'box', '2', 'demonstration', '0', '0')
        # learning.train_model()

    elif args.mode == 'demo':
        name = 'tracked_PPO2_mlp_default_classic'
        learning = LearningOpenAI('PPO2', 'classic', 
                'box', '2', 'demonstration', '0', '0')
        learning.perform_demonstration(name)
    elif args.mode == 'debug':
        LearningOpenAI.debug()

