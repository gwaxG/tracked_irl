#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import os
import gym
import time
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
import json, sys

class ExperimentBuilder:
    pass

class LearningOpenAI:
    def __init__(self, **kwargs):
        alg = kwargs['alg']
        comment = kwargs['comment']
        self.iteration = kwargs['iteration']
        # del kwargs['alg']
        # del kwargs['comment']
        # del kwargs['iteration']
        
        # alg, reward_type, action_type, action_num, comment, nesm, polygon, discretize=True):
        # 3 sets means that there will be tested lwo, medium and high parameter values
        # env = TrackedEnv(action_type, reward_type, action_num, nesm, polygon, discretize=True) 
        # env = gym.make("Tracked-v0", **{'action_type': action_type, 'reward_type': reward_type, 'action_num' : action_num, 'nesm': nesm, 'polygon': polygon, 'env_type': 'step', 'discretize': 'True'})
        self.name = "_".join([alg, kwargs["env_type"], kwargs["action_type"], kwargs["nesm"], kwargs["polygon"], comment])
        kwargs['log_address'] = self.get_path("custom_logs")
        env = gym.make("Tracked-v0", **kwargs)
        self.env = DummyVecEnv([lambda: env])
        self.stability_counter = 0
        self.projection_counter = 0
        self.test_counter = 0
        self.cur_ep = 0
        self.step = 0
        self.set_callback = True
        if alg != 'DQN':
            n_actions = env.action_space.shape[-1]
            action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
            ou_action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))

        if alg == 'PPO2':
            # Work model
            self.model = PPO2(PPO2MlpPolicy, self.env, n_steps=128, verbose=1, learning_rate=1e-4,
               tensorboard_log=self.get_path('logs'))
            # We check how big the reward is without learning
            # self.tests()

            # Test
            # self.model = PPO2(PPO2MlpPolicy, self.env, n_steps=5, verbose=1, nminibatches=1, tensorboard_log=self.get_path('logs'), learning_rate=5e-5)

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

    def get_path(self, identificator):
        path = os.path.dirname(os.path.realpath(__file__))
        path = "/".join(path.split("/")[:-1])
        path = os.path.join(path, identificator)
        if identificator == "logs":
            path = os.path.join(path, self.name)
        else:
            path = os.path.join(path, self.name+"_"+str(self.iteration))
        return path

    def train_model(self, time_steps=20000):
        for i in range(1):
            if i > 0:
                # Candidate to be deleted
                self.model.load(self.get_path('learning_results'))
            if self.set_callback:
                self.model.learn(total_timesteps=time_steps, callback=self.callback)
            else:   
                self.model.learn(total_timesteps=time_steps)

            self.model.save(self.get_path('learning_results'))

    def callback(self, locals_, globals_):
        # print('\nCALLING CALLBACK\n')
        self_ = locals_['self']
        ######### Performing tests ##########
        # print('\n Do I enter test section ? \n', self.test_counter, 2, self.test_counter % 2 == 0)
        if self.test_counter == 1 or \
                self.test_counter == 10 or \
                self.test_counter % 8 == 0:
            # print('I entered test section')
            test_results = self.tests(self_)
            # print('Tests done')
            for k, v in test_results.items():
                # print('Saving summary for tests', k)
                summary = tf.Summary(
                    value=[tf.Summary.Value(tag=k+'_test_result',
                                                            simple_value=v)])
                locals_['writer'].add_summary(summary, self_.num_timesteps)
            print('Exiting test section')

        self.test_counter += 1
        #####################################
        env_ep = self_.env.get_attr('ep_num')[0]
        self.stability_counter += self_.env.get_attr('stability_per_step')[0]
        self.projection_counter += self_.env.get_attr('projection_per_step')[0]
        self.step += 1
        # print('\n\n\nTrying to write data for projection and stability\n\n\n', self.cur_ep, env_ep)
        if self.cur_ep != env_ep:
            self.cur_ep = env_ep
            print('Episode {} stabilitiy {}'.format(self.cur_ep-1, self.stability_counter))
            summary = tf.Summary(value=[tf.Summary.Value(tag='instability_per_episode', simple_value=self.stability_counter/self.step)])
            locals_['writer'].add_summary(summary, self_.num_timesteps)
            summary = tf.Summary(value=[tf.Summary.Value(tag='projection_per_episode', simple_value=self.projection_counter/self.step)])
            locals_['writer'].add_summary(summary, self_.num_timesteps)
            self.projection_counter = 0
            self.stability_counter = 0
            self.step = 0
        return True

    def tests(self, self_):
        results = {
            'reward': 0,
            'stability': 0,
            'projection': 0
        }
        for k, v in self_.env.get_attr('env_creator')[0].HL_test.items():
            self_.env.set_attr('communicator', v)
            obs = self.env.reset()
            cnt = 0
            r = 0
            p = 0
            s = 0
            while True:
                cnt += 1
                action, _states = self.model.predict(obs)
                obs, reward, done, info = self_.env.get_attr('step')[0](action, True)
                r += reward
                p += info['projection']
                s += info['stability']
                if done:
                    break
            results['reward'] += r
            results['stability'] += s / cnt
            results['projection'] += p / cnt
        self_.env.set_attr('communicator', None)

        num_exps = len(self_.env.get_attr('env_creator')[0].HL_test.keys())
        for k in results.keys():
            results[k] /= num_exps
        return results




    def __del__(self):
        print("DELETING INSTANCE")

    @staticmethod
    def debug():
        path = os.path.abspath('..')
        f = open(os.path.join(path, "plan.json"))
        kwargs = json.load(f)
        k = list(kwargs.keys())
        kwargs = kwargs[k[0]]
        del kwargs['alg']
        del kwargs['comment']
        del kwargs['repeat']
        env = gym.make("Tracked-v0", **kwargs)
        # while True:
            # print(env.stability_estimation())

    def perform_demonstration(self):
        name = 'PPO2_step_box_1_0_official_exp_nesm_0.zip'
        print('Performing demonstration, /{} is loaded'.format(name))
        self.model.load(self.get_path('learning_results', name))
        obs = self.env.reset('big')
        while True:
            action, _states = self.model.predict(obs)
            obs, rewards, done, info = self.env.step(action)
            if done:
                self.reset('big')

        self.env.close()

def perform_demonstration():
    path = os.path.dirname(os.path.realpath(__file__))

    sys.path.append("../")
    from Learning.utils.env_creator import Creator
    env_creator = Creator()
    sys.path.append("/wcci2020/learning_scripts")
    name = 'PPO2_step_box_1_0_official_exp_nesm_1.zip'
    model_path = os.path.join(os.getcwd(),  'learning_results')
    model_path = os.path.join(model_path, name)
    model = PPO2.load(model_path)

    f = open(os.path.join(os.getcwd(), "plan.json"))
    kwargs = json.load(f)['demonstration']
    kwargs["env_creator"] = env_creator
    env = gym.make("Tracked-v0", **kwargs)
    state = env.reset('medium')
    env.base_control.reset()
    while True:
        actions = model.predict(state)
        state, r, done, info = env.step(list(actions[0]))
        if done:
            state = env.reset('medium')
            print('Env reseted')

if __name__ == '__main__':
    perform_demonstration()
