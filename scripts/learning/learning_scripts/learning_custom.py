#!/usr/bin/env python
# -*- coding: utf-8 -*-


import math
import numpy as np
import gym
import torch
import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from gym import spaces
from gym.utils import seeding
import matplotlib.pyplot as plt
import os, sys
# from actors.common_actor2 import PolicyNetWork
from actors.common_actor2 import PolicyNetWork
from actors.actor_sarsa import QNetWork
from actors.actor_reps import PolicyREPS

from critics.common_critic import ValueNetWork

from algorithms.reinforce import REINFORCE
from algorithms.reps import REPS
from algorithms.sarsa import SARSA
from custom_env.env import Env
import message_filters
import time
from multiprocessing import Pool
from itertools import product

'''
0     push left
1     push right
pid: {p: 20.0, i: 0.1, d: 0.5}
pid: {p: 1000.0, i: 200.0, d: 200.0}
'''

class Learning:

    def __init__(self):
        self.env = Env()
        self.counter = 0
        self.alg = 'sarsa'
        self.possible_actions = list(product([1, 0, -1], repeat=2))

        self.sync = False
        # Number of observation dimensions
        observ_space = self.env.observation_space
        # Number of action dimensions
        act_space = self.env.action_space
        #  it was 96
        hidden = 128
        if self.alg == 'reinforce':
            self.actor = PolicyNetWork(observ_space, act_space, hidden)
        elif self.alg == 'reps':
            self.actor = PolicyREPS(observ_space, act_space, hidden)
        elif self.alg == 'sarsa':
            self.actor = QNetWork(observ_space+1, act_space, hidden)


        # critic's input is observation_space + 1 where 1 is for the action
        # for reps only
        self.critic = ValueNetWork(observ_space+act_space, 1, hidden)
        self.algorithms = {
                            'reps': REPS(),
                            'reinforce': REINFORCE(),
                            'sarsa': SARSA()
                            }
        self.actor_critic = False

    def show(self, data, labels):
        plt.figure(1)
        nmbs = [221, 222, 223, 224]
        for i in range(len(data)):
            nmb = nmbs[i]
            dat = data[i] # self.smooth(data[i])
            lbl = labels[i]
            plt.subplot(nmb)
            plt.plot(dat)
            plt.title(lbl, fontsize=8)
            plt.grid(True)
        # smoothed_rewards = [np.mean(rewards[i-window:i+1]) if i > window else np.mean(rewards[:i+1]) for i in range(len(rewards))]
        plt.show()

    def smooth(self, scatters, win=10):
        scat = []
        scatters = np.array(scatters)
        for i in range(len(scatters)-win):
            scat.append(np.mean(scatters[i:i+win]))
        j = win
        for i in range(len(scatters)-win, len(scatters)):
            scat.append(np.mean(scatters[i:i+j]))
            j -= 1
        return scat

    def run(self):
        samples = []
        # Choose algorithm
        # alg = 'reinforce'
        algorithm = self.algorithms[self.alg]
        for episode in range(700):
            state = self.env.reset()
            print('#######\nEpisode {}'.format(episode))
            if self.alg == 'sarsa':
                action, probs = self.actor.act(state)
            for _ in range(2000):
                # Choose an action in episodic-based setting
                ############################################
                if self.alg == 'reinforce':
                    action, log_probs = self.actor.act(state,'reinforce')
                elif self.alg == 'reps':
                    action = self.actor.act(state)
                ############################################
                #  Perform the action and obtain observations
                prev_state = state
                if self.alg == 'sarsa':
                    actions = list(self.possible_actions[action])
                    state, reward, done, info = self.env.step(actions)
                else:
                    state, reward, done, info = self.env.step(action)
                ############################################
                #  reinforce and reps just append observations with needed information
                if self.alg == 'reinforce':
                    samples.append([prev_state, action, state, reward, log_probs])
                elif self.alg == 'reps':
                    samples.append([prev_state, action, state, reward])
                ############################################
                # sarsa needs to update just after getting observations and CHOOSE ACTION
                elif self.alg == 'sarsa':
                    self.actor, action, probs = algorithm.update(self.actor, [prev_state, action, state, reward, probs])
                if done:
                    if self.alg == 'sarsa':
                        algorithm.reward.append(algorithm.r)
                        print("Reward: {}".format(algorithm.r))
                        algorithm.r = 0
                    break
            ############################################
            # Episode-based algorithms performs their updates of networks
            if self.alg == 'reinforce':
                self.actor = algorithm.update_policy(self.actor, samples)
            elif self.alg == 'reps':
                self.actor, self.critic = algorithm.update_policy(self.actor, self.critic, samples)
            samples = []
        self.env.close()
        if self.alg == 'reps':
            dat = [algorithm.loss_actor, algorithm.reward, algorithm.loss_critic, algorithm.eta]
            labels = ['loss actor', 'reward per episode', 'loss critic', 'eta']
        elif self.alg == 'reinforce':
            self.actor.save_weights()
            dat = [algorithm.loss_actor, algorithm.reward]
            labels = ['loss actor', 'reward per episode']
        elif self.alg == 'sarsa':
            dat = [algorithm.reward]
            labels = ['reward per episode']
        self.show(dat, labels)

    # To delete
    def hinge_callback(self, hinge_states):
        # print('Hinge callback')
        self.env.base_control.update_hinges(hinge_states)
        self.sync = True

    def gazebo_callback(self, model_state):
        self.counter += 1
        # Updates every 100 ms
        rate = 100
        if float(self.counter) % rate == 0:
            self.env.update_model_state(model_state)
            self.counter = 0

    def test(self, type_='reinforce'):
        path = os.path.dirname(os.path.realpath(__file__)) + '/learning_results/1_2_128_perceptron'
        self.actor.load_state_dict(torch.load(path))
        get_actions = lambda probs: [int(torch.argmax(prob, dim=1))-1 for prob in probs]
        state = self.env.reset()
        for _ in range(2000):
            actions = get_actions(self.actor(state))
            state, reward, done, info = self.env.step(actions)
            time.sleep(0.3)
        self.env.close()

def main():
    # process.run()
    mode = sys.argv[1]
    print('Learning node is started')
    process = Learning()
    rospy.init_node('Learning', anonymous=True)
    # rospy.Subscriber('/tracked_robot/joint_states', JointState, process.hinge_callback, queue_size=10)
    rospy.Subscriber('/gazebo/link_states', LinkStates, process.gazebo_callback, queue_size=1)
    pool = Pool(10)
    if mode == 'train':
        pool.map(process.run(), ())
    elif mode == 'test':
        pool.map(process.test(), ())
    else:
        print('No mode, ended')
        return None
    # process.run()
    rospy.Rate(100)
    rospy.spin()

if __name__ == '__main__':
    main()
