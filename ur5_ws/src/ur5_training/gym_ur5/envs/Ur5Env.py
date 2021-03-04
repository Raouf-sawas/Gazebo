#!/usr/bin/env python333

# Author: Abdulraouf

#!/usr/bin/env python33
import sys
import rospy
import moveit_commander
import collections
import geometry_msgs.msg
import numpy as np
import random
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import argparse
import actionlib
import control_msgs.msg


import rospy

import rospy


#force_tourqesensors
from geometry_msgs.msg._WrenchStamped import WrenchStamped
#force_tourqesensors

import rospy
from nav_msgs.msg import Odometry
import gym
import rospy
import roslaunch
import time
import numpy as np
from gym import utils,spaces
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from gym.utils import seeding
import gym
import rospy
import time

import tf
import time
from gym import utils,spaces
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty as EmptyTopicMsg
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection
from gym.utils import seeding
import random
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import argparse
import actionlib
import control_msgs.msg


import random
import sys
sys.path.insert(0, '..')
import os
import time
import math
import  cv2 as cv
import numpy as np

from gym import utils, spaces

import gym
from gym import spaces
reg=register(
id='UR5env-v0',
entry_point='UR5_env:UR5Env',)


class UR5Env(gym.Env):
    def __init__(self):
        super(UR5Env, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_group_python_interface',anonymous=True)
        self.unpause=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        self.pause=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        self.reset_proxy=rospy.ServiceProxy('/gazebo/reset_simulation',Empty)
        self.robot=moveit_commander.RobotCommander()
        self.arm_group=moveit_commander.MoveGroupCommander("arm")

        self.depth_axis,self.elvation_dis,self.lateral_dist= 0,0,0
        self.f_t =[0,0,0,0,0,0]
        self.privous_coord = [0, 0, 0]
        self.init = [0.43, 0.04, 0.5]
        self.coord = [0.43, 0.4, 0.5]
        self.target = [0.47, 0.0, 0.522]

        self.done = False
        self.target_reached = 3
        self.reward = 0
        self.epoch = 0
        self.wait = 20
        self.step_size = 0.001
        self.action_space = spaces.Discrete(17)
        self.observation_space = spaces.Box(low=-100, high=100, shape=(6,), dtype=np.float32)

def ft_sensors_listener_filter(self):
    # print("ft_sensors_listener_filter")
    msg = rospy.wait_for_message("/ft_sensor/raw",WrenchStamped)
    ft_data=[0,0,0,0,0,0]
    for i in range(50):
        print("force_torque:",msg.wrench.force , msg.wrench.torque)
        ft_data += [msg.wrench.force , msg.wrench.torque]
    force_torque = [i / 50 for i in ft_data]
    ft = [round(i, 4) for i in force_torque]
    ft[1]=ft[1]-15
    if(np.isnan(f_t[0])):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
    print("force_torque:",ft)
    return ft


    def calculate_euclidean_distances(self):

        #distance between new self.coordinate and last coordinate

        self.dist_T_O = [np.linalg.norm(self.target[i]-self.privous_coord[i]) for i in range(3)]

        #distance between new coordinate and target
        self.dist_T_N = [np.linalg.norm(self.target[i]-self.coord[i]) for i in range(3)]
        print("dist_T_N",self.dist_T_N)

        #define depth axis, elvation_dis, lateral_dist
        self.lateral_dist = np.linalg.norm(self.target[self.geoAxes[0]]-self.coord[self.geoAxes[0]])
        self.depth_axis = self.coord[self.geoAxes[1]]
        self.elvation_dis = np.linalg.norm(self.target[self.geoAxes[2]]-self.coord[self.geoAxes[2]])

    def step(self, action):

        self.reward = 0
        self.privous_coord = self.coord.copy()
        self.move(action)
        self.calculate_euclidean_distances()
        self.controller.stay(self.wait)
        #self.f_t = x.append(i) for i in self.ft_sensors_listener_filter()
        #self.f_t = [number / 10 for number in self.controller.get_ft()]
        self.check_collision()
        self.check_approching_target()
        self.check_target()


        state = [0,0,0,0,0,0]
        state =  self.f_t

        print("state",state )
        self.reward = round(self.reward, 2)
        print("REWARD = ",self.reward)
        print("="*30)
        self.epoch += 1
        print("EPOCH=",self.epoch)
        if(self.epoch > 350):
            self.epoch = 0
            #self.reward -= 20
            self.done = True


        return np.array(state, dtype=np.float32), self.reward, self.done, {}

    def reset(self):

        print("*"*30)
        self.done = False
        self.reward = 0
        self.epoch = 0
        #self.coord =  self.init
        print("*** x, y, z =",self.coord[0], self.coord[1], self.coord[2],  "***")
        self.geoAxes=[1,0,2]
        self.depth_target= 0.46
        pose_targe.orientation.w = -0.5
        pose_targe.orientation.x = 0.5
        pose_targe.orientation.y = -0.5
        pose_targe.orientation.z = 0.5

        pose_targe.position.x = 0.55
        pose_targe.position.y = 0.0
        pose_targe.position.z = 0.52
        self.arm_group.set_max_velocity_scaling_factor(1)
        self.arm_group.set_max_acceleration_scaling_factor(1)
        self.arm_group.set_pose_target(pose_targe)
        plan1 = self.arm_group.go()


        if self.target_reached   == 3:
            self.target_reached = 0
            self.target[0]=random.uniform(-0.1,0.1)
            self.init[0] = random.uniform(self.target[0] - 0.006, self.target[0] + 0.006)
            self.init[1] = random.uniform(-0.436, -0.433)
            self.init[2] = random.uniform(1.11, 1.117)
            self.controller.change_object_palace(self.target[0], -.75, 0.95,"platt")
        self.coord = self.init.copy()

        self.degree = ((self.coord[0]+0.0001)/0.0001) * 0.0135
        print("correction_value=",self.degree)
        #self.target_reached = False

        self.arm_group.set_pose_target(pose_targe)
        plan1 = self.arm_group.go()
        self.f_t = self.ft_sensors_listener_filter()

        #Unpausesimulationtomakeobservation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause=pause.call()
                self.unpause()
        except(rospy.ServiceException) as e:
            print("/gazebo/unpause_physicsservicecallfailed")

        return np.array( self.f_t, dtype=np.float32)

    def check_collision(self):
        collision = sum([True  if self.f_t[i] > 40 \
             else True if self.f_t[i] < -40 \
                 else False for i in range(6)])
        if collision > 0 :
            self.done= True
            self.reward -= 20
            print("*"*10, "COLLISION", "*"*10)

        self.reward += sum([-1  if self.f_t[i] > 30 \
             else -1 if self.f_t[i] < -30 \
                 else 1 for i in range(3)])
        print("force REWARD: = ", self.reward)
        #self.reward +=sum ([np.abs(self.f_t[i]) /r1  if self.f_t[i] < 20  and  self.f_t[i] > -20 \
        #    and sum(self.dist_T_N) <   0.005 and sum(self.dist_T_N) < sum(self.dist_T_O) \
        #    else 0 for i in range(3)])
        self.reward += sum ([1  if self.f_t[i] < 30  and  self.f_t[i] > -30 \
            and sum(self.dist_T_N) <   0.01 and sum(self.dist_T_N) < sum(self.dist_T_O) \
            else -1 for i in range(3)])
        print("force REWARD: = ", self.reward)

    def check_target(self):
        if self.elvation_dis > 0.02  or self.lateral_dist  > 0.02:
            self.done = True
            self.reward -= 100
            print(10*"*","WENT OUT OF BORDERS",10*"*")
            print("self.elvation_dis > 0.015  or self.late",self.elvation_dis, self.lateral_dist)

        if np.abs(self.depth_axis) >= np.abs(self.depth_target) and self.elvation_dis < 0.002  and self.lateral_dist  < 0.002 :
            self.done = True
            self.reward += 100
            self.target_reached += 1
            print(10*"*","TASK ACCOMPLISHED",10*"*")

    def check_approching_target(self):
        print("REWARD: = ", self.reward)

        d_p = 0.0001
        r1 = 0.002
        r2 = 0.002
        print("approaching REWARD: = ", self.reward)

        print("sum(self.dist_T_N) < sum(self.dist_T_O)",sum(self.dist_T_N), sum(self.dist_T_O))
        print("#"*30)

        #self.reward +=sum ([(r1 / (self.dist_T_N [i]+d_p)) if self.dist_T_N [i] < self.dist_T_O[i]  \
        #    else (r2 / (self.dist_T_N [i]+d_p)) if  sum(self.dist_T_N) < sum(self.dist_T_O)   \
        #        else -(r1 / (self.dist_T_N [i]+d_p)) for i in range(3)])
        self.reward += sum([ 2 if  self.dist_T_N[i] < self.dist_T_O[i] \
            else 0 if  self.dist_T_N[i] == self.dist_T_O[i] else -2 for i in range(3)])
        print("approaching REWARD: = ", self.reward)
        self.reward += (-2 if np.abs(self.coord[self.geoAxes[1]]) < np.abs(self.privous_coord[self.geoAxes[1]]) and sum(self.dist_T_N) <= 0.01 \
                         else 4 if sum(self.dist_T_N) <= 0.01 else 0 )
        print("approaching REWARD: = ", self.reward)
        print("#"*30)
        print("#"*30)
        for i in range(3):
            print(r1 / (self.dist_T_N [i]+d_p))
            print(r2 / (self.dist_T_N [i]+d_p))
        print("#"*30)

    def move(self, action):


        self.movment = [0,0,0]
        if (action == 0):#Movingonxaxis X
            self.coord[0] += self.step_size
            self.movment[0] = 1

        elif (action == 1):#Movingonx axis X
             self.coord[0] -= self.step_size
             self.movment[0] = 0.5

        elif (action == 2):##Movingony axis Y
            self.coord[1] -= self.step_size
            self.movment[0] = 0.5

        elif (action == 3):##Movingonz axis Z
             self.coord[2] += self.step_size
             self.movment[2] = 1

        elif (action == 4):##Movingonz axis Z
             self.coord[2] -= self.step_size

        elif (action == 5):##Movingonz axis XY
            self.coord[0] -= self.step_size
            self.coord[1] -= self.step_size

        elif (action == 6):##Movingonz axis XY
            self.coord[0] += self.step_size
            self.coord[1] -= self.step_size

        elif (action == 7):##Movingonz axis XZ
            self.coord[0] += self.step_size
            self.coord[2] += self.step_size

        elif (action == 8):##Movingonz axis XZ
            self.coord[0] -= self.step_size
            self.coord[2] -= self.step_size

        elif (action == 9):##Movingonz axis XZ
            self.coord[0] += self.step_size
            self.coord[2] -= self.step_size
            self.movment = [1,0,0.5]
            self.degree += self.correction_value

        elif (action == 10):##Movingonz axis XZ
            self.coord[0] -= self.step_size
            self.coord[2] += self.step_size

        elif (action == 11):##Movingonz axis YZ
            self.coord[1] -= self.step_size
            self.coord[2] -= self.step_size

        elif action == 12 :##Movingonz axis YZ
            self.coord[1] -= self.step_size
            self.coord[2] += self.step_size

        elif action == 13 :##Movingonz axis XYZ
            self.coord[0] += self.step_size
            self.coord[1] -= self.step_size
            self.coord[2] -= self.step_size

        elif action == 14 :##Movingonz axis XYZ
            self.coord[0] -= self.step_size
            self.coord[1] -= self.step_size
            self.coord[2] += self.step_size
            self.movment = [0.5,0.5,1]
            self.degree -= self.correction_value

        elif action == 15 :##Movingonz axis XYZ
            self.coord[0] -= self.step_size
            self.coord[1] -= self.step_size
            self.coord[2] -= self.step_size

        elif action >= 16 :##Movingonz axis XYZ
           self.coord[0] += self.step_size
           self.coord[1] -= self.step_size
           self.coord[2] += self.step_size

        print("SELECTED ACTION  = ",action)
        print("cor:", self.coord)
        pose_targe = geometry_msgs.msg.Pose()
        pose_targe.orientation.w = -0.5
        pose_targe.orientation.x = 0.5
        pose_targe.orientation.y = -0.5
        pose_targe.orientation.z = 0.5
        pose_targe.position.x = self.coord[0]
        pose_targe.position.y = self.coord[1]
        pose_targe.position.z = self.coord[2]

        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.1)
        self.arm_group.set_pose_target(pose_targe)
        plan1 = self.arm_group.go()
