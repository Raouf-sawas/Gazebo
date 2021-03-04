import gym
from stable_baselines import DQN, PPO2, A2C, ACKTR, SAC
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.cmd_util import make_vec_env
import numpy as np
#from stable_baselines.deepq.policies import MlpPolicy, CnnPolicy
import rospy
import os
import time
#from gym_ur5.controller.MujocoController import MJ_Controller
#env = gym.make('gym_ur5:UR5-v0')
rospy.init_node('UR5env_algorithem', anonymous=True, log_level=rospy.FATAL)
env = make_vec_env('gym_ur5:UR5-v0', n_envs=1)
# wrap it
#env = make_vec_env(lambda: env, n_envs=4)
# Train the agent
model = A2C(MlpPolicy, env, verbose=1,  tensorboard_log="./a2c_cartpole_tensorboard/")
#model.learn(steps_per_epoch=5000, epochs=100)
#model.learn(total_timesteps=1000000)
#del model
#model.save("Ur5A2C")
model = A2C.load("Ur5A2C")
# Test the trained agent
obs = env.reset()
n_steps = 1000
for step in range(n_steps):
  action, _ = model.predict(obs, deterministic=True)
  print("Step {}".format(step + 1))
  print("Action: ", action)
  obs, reward, done, info = env.step(action)
  print('obs=', obs, 'reward=', reward, 'done=', done)
