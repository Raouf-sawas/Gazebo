#!/usr/bin/env python


import gym
import time
from std_srvs.srv import Empty
import numpy
import random
import time
import qlearn
from gym import wrappers

# ROS packages required
import rospy
import rospkg

# import our training environment
import UR5_env


import random
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import argparse
import actionlib
import control_msgs.msg

import gym
from gym import wrappers
import gym_gazebo
import time
import numpy
import random
import time

import qlearn
import liveplot


import gym
import rospy
import roslaunch
import time
import numpy as np

from gym import utils, spaces

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

from gym.utils import seeding
import rospy
 
# Inspired by https://keon.io/deep-q-learning/
import random
import gym
import math
import numpy as np
from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
 
# import our training environment
from openai_ros.task_envs.cartpole_stay_up import stay_up
 
class DQNRobotSolver():
    def __init__(self, environment_name, n_observations, n_actions, n_episodes=1000, n_win_ticks=195, min_episodes= 100, max_env_steps=None, gamma=1.0, epsilon=1.0, epsilon_min=0.01, epsilon_log_decay=0.995, alpha=0.01, alpha_decay=0.01, batch_size=64, monitor=False, quiet=False):
        self.memory = deque(maxlen=100000)
        self.env = gym.make(environment_name)
        if monitor: self.env = gym.wrappers.Monitor(self.env, '../data/cartpole-1', force=True)
        
        self.input_dim = n_observations
        self.n_actions = n_actions
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_log_decay
        self.alpha = alpha
        self.alpha_decay = alpha_decay
        self.n_episodes = n_episodes
        self.n_win_ticks = n_win_ticks
        self.min_episodes = min_episodes
        self.batch_size = batch_size
        self.quiet = quiet
        if max_env_steps is not None: self.env._max_episode_steps = max_env_steps
 
        # Init model
        self.model = Sequential()
        
        self.model.add(Dense(24, input_dim=self.input_dim, activation='tanh'))
        self.model.add(Dense(48, activation='tanh'))
        self.model.add(Dense(self.n_actions, activation='linear'))
        self.model.compile(loss='mse', optimizer=Adam(lr=self.alpha, decay=self.alpha_decay))
    
 
 
    def get_epsilon(self, t): 
        return max(self.epsilon_min, min(self.epsilon, 1.0 - math.log10((t + 1) * self.epsilon_decay))) 
    def preprocess_state(self, state): 
        return np.reshape(state, [1, self.input_dim]) 
    def replay(self, batch_size): x_batch, y_batch = [], [] 
    minibatch = random.sample( self.memory, min(len(self.memory), batch_size)) 
        for state, action, reward, next_state, done in minibatch: y_target = self.model.predict(state) y_target[0][action] = reward if done else reward + self.gamma * np.max(self.model.predict(next_state)[0]) x_batch.append(state[0]) y_batch.append(y_target[0]) self.model.fit(np.array(x_batch), np.array(y_batch), batch_size=len(x_batch), verbose=0) if self.epsilon &gt; self.epsilon_min:
            self.epsilon *= self.epsilon_decay
 
    def run(self):
        
        rate = rospy.Rate(30)
        
        scores = deque(maxlen=100)
 
        for e in range(self.n_episodes):
            
            init_state = self.env.reset()
            
            state = self.preprocess_state(init_state)
            done = False
            i = 0
            while not done:
                # openai_ros doesnt support render for the moment
                #self.env.render()
                action = self.choose_action(state, self.get_epsilon(e))
                next_state, reward, done, _ = self.env.step(action)
                next_state = self.preprocess_state(next_state)
                self.remember(state, action, reward, next_state, done)
                state = next_state
                i += 1
                
 
            scores.append(i)
            mean_score = np.mean(scores)
            if mean_score &gt;= self.n_win_ticks and e &gt;= min_episodes:
                if not self.quiet: print('Ran {} episodes. Solved after {} trials'.format(e, e - min_episodes))
                return e - min_episodes
            if e % 1 == 0 and not self.quiet:
                print('[Episode {}] - Mean survival time over last {} episodes was {} ticks.'.format(e, min_episodes ,mean_score))
 
            self.replay(self.batch_size)
            
 
        if not self.quiet: print('Did not solve after {} episodes'.format(e))
        return e
        
if __name__ == '__main__':
    rospy.init_node('cartpole_n1try_algorithm', anonymous=True, log_level=rospy.FATAL)
    
    environment_name = 'UR5env-v0'
    
    n_observations = rospy.get_param('/UR5env_v0/n_observations')
    n_actions = rospy.get_param('/UR5env_v0/n_actions')
    
    n_episodes = rospy.get_param('/UR5env_v0/episodes_training')
    n_win_ticks = rospy.get_param('/UR5env_v0/n_win_ticks')
    min_episodes = rospy.get_param('/UR5env_v0/min_episodes')
    max_env_steps = None
    gamma =  rospy.get_param('/UR5env_v0/gamma')
    epsilon = rospy.get_param('/UR5env_v0/epsilon')
    epsilon_min = rospy.get_param('/UR5env_v0/epsilon_min')
    epsilon_log_decay = rospy.get_param('/UR5env_v0/epsilon_decay')
    alpha = rospy.get_param('/UR5env_v0/alpha')
    alpha_decay = rospy.get_param('/UR5env_v0/alpha_decay')
    batch_size = rospy.get_param('/UR5env_v0/batch_size')
    monitor = rospy.get_param('/UR5env_v0/monitor')
    quiet = rospy.get_param('/UR5env_v0/quiet')
    
      qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                    alpha=0.2, gamma=0.8, epsilon=0.9)
    agent = DQNRobotSolver(     environment_name,
                                n_observations,
                                n_actions,
                                n_episodes,
                                n_win_ticks,
                                min_episodes,
                                max_env_steps,
                                gamma,
                                epsilon,
                                epsilon_min,
                                epsilon_log_decay,
                                alpha,
                                alpha_decay,
                                batch_size,
                                monitor,
                                quiet)
    agent.run()



import gym
import time
from std_srvs.srv import Empty
import numpy
import random
import time
import qlearn
from gym import wrappers

# ROS packages required
import rospy
import rospkg

# import our training environment
import UR5_env


import random
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import argparse
import actionlib
import control_msgs.msg

import gym
from gym import wrappers
import gym_gazebo
import time
import numpy
import random
import time

import qlearn
import liveplot


import gym
import rospy
import roslaunch
import time
import numpy as np

from gym import utils, spaces

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

from gym.utils import seeding

import rospy
import time

import random
import gym
import math
import numpy as np
from collection import deque
from keras.layers import Dense
from keras.models import Sequential
from keras.optimizers import Adam

from openai_ros.task.envs.turtulpot2 import turtulpot2_maze

class DQNRobotSolver():
 
    def __int__(self, environment_name, n_observations, n_actions, n_episodes=1500, gamma=1.0, epsilon=1.0, epsilon_min=0.01, epsilon_log_decay=0.995, alpha=0.01, alpha_decay=0.01, batch_size=64, monitor=False, quiet=False):
        
        self.env=gym.make(enviroment_name)
        self.memory=deque(maxlen=100000)
        self.env = gym.make(environment_name)
        if monitor: self.env = gym.wrappers.Monitor(self.env, '../data/ur5-1', force=True)
        
        self.input_dim = n_observations
        self.n_actions = n_actions
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_log_decay
        self.alpha = alpha
        self.alpha_decay = alpha_decay
        self.n_episodes = n_episodes
        #self.n_win_ticks = n_win_ticks
        #self.min_episodes = min_episodes
        self.batch_size = batch_size
        self.quiet = quiet
     
        # Init model
        self.model = Sequential()
            
        self.model.add(Dense(24, input_dim=self.input_dim, activation='tanh'))
        self.model.add(Dense(48, activation='tanh'))
        self.model.add(Dense(self.n_actions, activation='linear'))
        self.model.compile(loss='mse', optimizer=Adam(lr=self.alpha, decay=self.alpha_decay))



    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
 
    
    def choose_action(self, state, epsilon):
        return self.env.action_space.sample() if (np.random.random() <= epsilon) else np.argmax(self.model.predict(state)) 

    
    def get_epsilon(self, t): 
        return max(self.epsilon_min, min(self.epsilon, 1.0 - math.log10((t + 1) * self.epsilon_decay))) 

    def preprocess_state(self, state): 
        return np.reshape(state, [1, self.input_dim]) 

    def replay(self, batch_size): 
        x_batch, y_batch = [], [] 
        minibatch = random.sample( 
            self.memory, min(len(self.memory), batch_size)) 
        for state, action, reward, next_state, done in minibatch: 
            y_target = self.model.predict(state) 
            y_target[0][action] = reward if done else reward + self.gamma * np.max(self.model.predict(next_state)[0]) 
            x_batch.append(state[0])
            y_batch.append(y_target[0]) 
        
        self.model.fit(np.array(x_batch), np.array(y_batch), batch_size=len(x_batch), verbose=0) 
        if self.epsilon &gt; self.epsilon_min:
           self.epsilon *= self.epsilon_decay

    def save(self,model_name):
        rospack=rospkg.RosPack()
        pkg_path= rospkg.get_path("ur5_training")
        outdir= pkg_path+"/models"
        if not os.path.exisits(outdir):
            os.mkedirs(outdir)
            rospy.logdebug("Created folder"+str(outdir))
        model_name_yaml_format =model_name+".yaml"
        model_name_HDF5_format =model_name+".h5"

        model_name_yaml_format_path =os.path.join(outdir,model_name_yaml_format)
        model_name_HDF5_format_path =os.path.join(outdir,model_name_HDF5_format)

        model_yaml =self.model.to_yaml()

        with open(model_name_yaml_format_path,"w") as yaml_file:
            yaml_file.write(model_yaml)

        self.model_name_yaml_format_path.save_weights(model_name_HDF5_format_path)
        print("Saved model to disk")



    def run(self):
        
        rate = rospy.Rate(30)
        
        scores = deque(maxlen=100)
 
        for e in range(self.n_episodes):
            init_state = self.env.reset()
            state = self.preprocess_state(init_state)
            done = False
            i = 0
            while not done:
                # openai_ros doesnt support render for the moment
                #self.env.render()
                action = self.choose_action(state, self.get_epsilon(e))
                next_state, reward, done, _ = self.env.step(action)
                next_state = self.preprocess_state(next_state)
                self.remember(state, action, reward, next_state, done)
                state = next_state
                i += 1
                
 
            scores.append(i)
            mean_score = np.mean(scores)
            if e &gt;= min_episodes:
                if not self.quiet: print('Ran {} episodes. Solved after {} trials'.format(e, e - min_episodes))
                return e - min_episodes
            if e % 1 == 0 and not self.quiet:
                print('[Episode {}] - Mean survival time over last {} episodes was {} ticks.'.format(e, min_episodes ,mean_score))
 
            self.replay(self.batch_size)
            self.save("robot_maze")
            
 
        if not self.quiet: print('Did not solve after {} episodes'.format(e))
        return e


if __name__ == '__main__':
    rospy.init_node('UR5env_algorithem', anonymous=True, log_level=rospy.FATAL)
    
    environment_name = 'UR5env-v0'
    
    n_observations = rospy.get_param('/UR5env_v0/n_observations')
    n_actions = rospy.get_param('/UR5env_v0/n_actions')
    
    n_episodes = rospy.get_param('/UR5env_v0/episodes_training')
    #n_win_ticks = rospy.get_param('/UR5env_v0/n_win_ticks')
    #min_episodes = rospy.get_param('/UR5env_v0/min_episodes')
    max_env_steps = 300
    gamma =  rospy.get_param('/UR5env_v0/gamma')
    epsilon = rospy.get_param('/UR5env_v0/epsilon')
    epsilon_min = rospy.get_param('/UR5env_v0/epsilon_min')
    epsilon_log_decay = rospy.get_param('/UR5env_v0/epsilon_decay')
    alpha = rospy.get_param('/UR5env_v0/alpha')
    alpha_decay = rospy.get_param('/UR5env_v0/alpha_decay')
    batch_size = rospy.get_param('/UR5env_v0/batch_size')
    monitor = rospy.get_param('/UR5env_v0/monitor')
    quiet = rospy.get_param('/UR5env_v0/quiet')
    
    
    agent = DQNRobotSolver(     environment_name,
                                n_observations,
                                n_actions,
                                n_episodes,
                                gamma,
                                epsilon,
                                epsilon_min,
                                epsilon_log_decay,
                                alpha,
                                alpha_decay,
                                batch_size,
                                monitor,
                                quiet)
    agent.run()        