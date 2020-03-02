import numpy as np
import os, sys
import rospy
from garage.torch.algos import DDPG
from gazebo_msgs.msg import LinkStates, LinkState
from garage.experiment import LocalRunner, run_experiment
from garage.np.baselines import LinearFeatureBaseline
from garage.tf.envs import TfEnv
from garage.torch.policies import GaussianMLPPolicy

def gazebo_callback(msg):
    print('That works')


if __name__ == '__main__':
    rospy.init_node('Learning_OpenAI', anonymous=True)
    rospy.Subscriber('/gazebo/link_states', LinkStates, gazebo_callback, queue_size=10)            
    rospy.spin()  
