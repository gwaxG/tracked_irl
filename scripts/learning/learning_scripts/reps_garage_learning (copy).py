#!/home/r1d1/venv/bin/python
# -*- coding: utf-8 -*-

"""
This is an example to train a task with REPS algorithm.
Here it runs gym CartPole env with 100 iterations.
Results:
    AverageReturn: 100 +/- 40
    RiseTime: itr 10 +/- 5
"""

import gym
import gym_tracked
import os, sys

from garage.experiment import run_experiment
from garage.np.baselines import LinearFeatureBaseline
from garage.tf.algos import REPS
from garage.tf.envs import TfEnv
from garage.tf.experiment import LocalTFRunner
from garage.tf.policies import CategoricalMLPPolicy

def run_task(snapshot_config, *_):
    """Run task."""
    import gym_tracked
    with LocalTFRunner(snapshot_config=snapshot_config, max_cpus=1) as runner:
        # discrete, discrete_dqn, box
        action_type='discrete_dqn'
        # classic, safe ...
        reward_type='classic'
        # 2 - flippers, 3 - fl. + moving back and forward
        # 4 - flippers and each track
        action_num='2'
        # Is nesm stabiltiy criteria applied ?
        nesm='0'
        # Is support polygon criteria applied ?
        polygon='0'
        # Type of env either step or target
        # step considers relative position to the next step
        # target works with distance to the end position
        env_type='step'
        discretize=True
        env = TfEnv(gym.make('Tracked-v0', **{'action_type': action_type, 'reward_type': reward_type, 'action_num' : action_num, 'nesm': nesm, 'polygon': polygon, 'env_type': 'step', 'discretize': True}))
        policy = CategoricalMLPPolicy(env_spec=env.spec, hidden_sizes=[128, 128])

        baseline = LinearFeatureBaseline(env_spec=env.spec)

        algo = REPS(env_spec=env.spec,
                    policy=policy,
                    epsilon = 0.1,
                    # l2_reg_dual=0.0001,
                    # l2_reg_loss=0.0001, 
                    baseline=baseline,
                    max_path_length=100,
                    discount=0.99)

        runner.setup(algo, env)
        runner.train(n_epochs=200, batch_size=10, plot=False)

for i in range(3):
    run_experiment(
        run_task,
        snapshot_mode='none',
        seed=1,
    )
