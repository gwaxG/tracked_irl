from gym.envs.registration import register
 
register(
	id='Tracked-v0', 
    entry_point='gym_tracked.envs:TrackedEnv', 
    kwargs={'action_type': 'box',
            'reward_type': 'classic',
            'action_num' : '2',
            'nesm': '0',
            'polygon': '0',
            'sigma': '0.',
            'env_type': 'step',
            'discretize': True,
            'alg': None,
            'comment': None,
            'repeat': 1,
            'iteration': 0,
            'env_creator': None,
            'log_address': None
            }
)
