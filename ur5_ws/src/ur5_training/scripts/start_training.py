#!/usr/bin/env python


import os
import gym
import time
from std_srvs.srv import Empty
import numpy
import random
import time

from gym import wrappers

# ROS packages required
import rospy
import rospkg

# import our training environment
import UR5_env
from keras.models import model_from_yaml

import random
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import rospkg
import argparse
import actionlib
import control_msgs.msg
import collections
import gym
from gym import wrappers
import gym_gazebo
import time
import numpy
import random
import time

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

from keras.layers import Dense
from keras.models import Sequential
from keras.optimizers import Adam



class DQNRobotSolver():
    def __init__(self, environment_name, n_observations, n_actions, n_episodes=1500, gamma=1.0, epsilon=1.0, epsilon_min=0.01, epsilon_log_decay=0.995, alpha=0.01, alpha_decay=0.01, batch_size=128, monitor=False, quiet=False):
        self.env=gym.make(enviroment_name)
        self.env.max_episode_steps=300
        self.memory=collections.deque(maxlen=100000)
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
        #self.load_Modle("ur5_robot_modle")

        self.model.add(Dense(24, input_dim=self.input_dim, activation='tanh'))
        self.model.add(Dense(48, activation='tanh'))
        self.model.add(Dense(self.n_actions, activation='linear'))
        self.model.compile(loss='mse', optimizer=Adam(lr=self.alpha, decay=self.alpha_decay))



    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))


    def choose_action(self, state, epsilon):
        print(self.env.action_space.sample() if (np.random.random() <= epsilon) else np.argmax(self.model.predict(state)))
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
            y_target[0][action] = reward if done else reward + self.gamma * np.max(self.model.predict(next_state))
            x_batch.append(state[0])
            y_batch.append(y_target[0])

        self.model.fit(np.array(x_batch), np.array(y_batch), batch_size=len(x_batch), verbose=0)
        if self.epsilon > self.epsilon_min:
           self.epsilon *= self.epsilon_decay

    def save(self,model_name):
        rospack=rospkg.RosPack()
        #pkg_path= "/home/ros/thesis/ur5_ws/src/ur5_training"
        pkg_path= rospack.get_path("ur5_training")
        outdir= pkg_path+"/models"
        if not os.path.exists(outdir):
            os.mkedirs(outdir)
            rospy.logdebug("Created folder"+str(outdir))
        model_name_yaml_format =model_name+".yaml"
        model_name_HDF5_format =model_name+".h5"

        model_name_yaml_format_path =os.path.join(outdir,model_name_yaml_format)
        model_name_HDF5_format_path =os.path.join(outdir,model_name_HDF5_format)

        model_yaml =self.model.to_yaml()

        with open(model_name_yaml_format_path,"w") as yaml_file:
            yaml_file.write(model_yaml)

        self.model.save_weights(model_name_HDF5_format_path)
        print("Saved model to disk")

    def load_Modle(self,model_name):
    	rospack = rospkg.RosPack()
    	pkg_path =rospack.get_path("ur5_training")
    	model_dir_path =pkg_path +"/models/"
    	model_name_yaml_format =model_name+".yaml"
        model_name_HDF5_format =model_name+".h5"

    	model_name_yaml_format_path = os.path.join(model_dir_path,model_name_yaml_format)
    	model_name_HDF5_format_path = os.path.join(model_dir_path,model_name_HDF5_format)

    	yaml_file =open(model_name_yaml_format_path, "r")
    	loded_model_yaml= yaml_file.read()
    	yaml_file.close()
    	self.model = model_from_yaml(loded_model_yaml)
    	self.model.load_weights(model_name_HDF5_format_path)
    	print("loaded model from disk ")


    def run(self):
        rate = rospy.Rate(30)
        highest_reward = 0
        last_time_steps = numpy.ndarray(0)
        start_time = time.time()
        f= open("/home/ros/thesis/ur5_ws/src/ur5_training/data/ur5_training.txt","w+")
        f1= open("/home/ros/thesis/ur5_ws/src/ur5_training/data/ur5_training_outcome.txt","w+")
        t0= time.clock()
        f.write("{},{},{} \n".format("cumulated_reward","epsilon", "time"))
        f1.write("{},{} \n".format("reward","time"))



        for e in range(self.n_episodes):
            cumulated_reward = 0
            init_state = self.env.reset()
            #state = ''.join(map(str, init_state))
            state = self.preprocess_state(init_state)
            done = False

            step=1500

            for i in range(step):

                # Pick an action based on the current state
                print("step",i)


#                action = qlearn.chooseAction(state)
                action = self.choose_action(state, self.get_epsilon(e))

                # Execute the action and get feedback
                observation, reward, done,  info = self.env.step(action)

                state=self.preprocess_state(observation)
                t0=int(time.time() - start_time)
                f1.write("{}, {} \n".format(reward, t0))

                cumulated_reward += reward

                if highest_reward < cumulated_reward:
                    highest_reward = cumulated_reward

                #next_state = ''.join(map(str, observation))
                next_state = self.preprocess_state(observation)

                self.remember(state, action, reward, next_state, done)
                #state = next_state
                if not(done):
                    state = next_state
                else:
                    last_time_steps = numpy.append(last_time_steps, [int(i + 1)])
                    break
            m, s = divmod(int(time.time() - start_time), 60)
            h, m = divmod(m, 60)
            t0=int(time.time() - start_time)
            f.write("{}, {} \n".format(cumulated_reward,round(self.epsilon,2),t0))
            print ("EP: "+str(e+1)+" - [alpha: "+str(round(self.alpha,2))+" - gamma: "+str(round(self.gamma,2))+" - epsilon: "+str(round(self.epsilon,2))+"] - Reward: "+str(cumulated_reward)+"     Time: %d:%02d:%02d" % (h, m, s))
            self.replay(self.batch_size)
            self.save("ur5_robot_modle")
        f.close()
        f1.close()


        print ("\n|"+str(self.n_episodes)+"|"+str(self.alpha)+"|"+str(self.gamma)+"|"+str(self.epsilon)+"*"+str(self.epsilon_decay)+"|"+str(highest_reward)+"| PICTURE |")
        return e



if __name__ == '__main__':
    #rospy.init_node('start_training', anonymous=True, log_level=rospy.FATAL)

    enviroment_name = 'UR5env-v0'

    n_observations = rospy.get_param('/UR5env_v0/n_observations')
    n_actions = rospy.get_param('/UR5env_v0/n_actions')

    n_episodes = rospy.get_param('/UR5env_v0/nepisodes')
    #n_win_ticks = rospy.get_param('/UR5env_v0/n_win_ticks')
    #min_episodes = rospy.get_param('/UR5env_v0/min_episodes')
    max_env_steps = 300
    gamma =  rospy.get_param('/UR5env_v0/gamma')
    epsilon = rospy.get_param('/UR5env_v0/epsilon')
    epsilon_min = rospy.get_param('/UR5env_v0/epsilon_min')
    epsilon_decay = rospy.get_param('/UR5env_v0/epsilon_decay')
    alpha = rospy.get_param('/UR5env_v0/alpha')
    alpha_decay = rospy.get_param('/UR5env_v0/alpha_decay')
    batch_size = rospy.get_param('/UR5env_v0/batch_size')
    monitor = rospy.get_param('/UR5env_v0/monitor')
    quiet = rospy.get_param('/UR5env_v0/quiet')

    agent = DQNRobotSolver(     enviroment_name,
                                n_observations,
                                n_actions,
                                n_episodes,
                                gamma,
                                epsilon,
                                epsilon_min,
                                epsilon_decay,
                                alpha,
                                alpha_decay,
                                batch_size,
                                monitor,
                                quiet)
    print(agent.n_episodes)
    agent.run()
