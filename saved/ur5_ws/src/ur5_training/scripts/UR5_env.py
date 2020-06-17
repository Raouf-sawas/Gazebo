#!/usr/bin/env python
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

#!/usr/bin/envpython

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

reg=register(
id='UR5env-v0',
entry_point='UR5_env:UR5Env',)


class UR5Env(gym.Env):
    l_fx, l_fy, l_fz = collections.deque(maxlen=3),collections.deque(maxlen=3),collections.deque(maxlen=3)
    l_tx, l_ty, l_tz = collections.deque(maxlen=3),collections.deque(maxlen=3),collections.deque(maxlen=3)
    f_x, f_z, t_x, t_y, t_z, f_x_avg, f_y_avg, f_z_avg ,t_x_avg ,t_y_avg, t_z_avg = 0,0,0,0,0,0,0,0,0,0,0
    f_y=16
    p_x,lp_x, lp_y, p_y,lp_z, p_z = 0.5, 0.5, 0.1, 0.1, 0.435, 0.435
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_group_python_interface',anonymous=True)
        self.unpause=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        self.pause=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        self.reset_proxy=rospy.ServiceProxy('/gazebo/reset_simulation',Empty)
        self.robot=moveit_commander.RobotCommander()
        self.arm_group=moveit_commander.MoveGroupCommander("arm")
        self.action_space=spaces.Discrete(8)
        self.reward_range=(-np.inf,np.inf)
        self._seed()

    def state_observation(self, action):
        on_x_axis, on_y_axis, on_z_axis = 0,0,0
        # print("UR5Env.lp_x",UR5Env.lp_x)
        # print("UR5Env.lp_y",UR5Env.lp_y)
        # print("UR5Env.lp_z",UR5Env.lp_z)
        # print("=old  lp_x===  ", (0.711 % UR5Env.p_x))
        if( (action==0 or action ==1 or action==6 or action==7) and UR5Env.p_x > UR5Env.lp_x  and UR5Env.p_x < 0.712 ):
        	on_x_axis = 1/((0.711 % UR5Env.p_x)+1)
        else:
            on_x_axis = -1/((0.711 % UR5Env.p_x)+1)
        UR5Env.lp_x = UR5Env.p_x
        # print("=?UR5Env.lp_x===  ", UR5Env.lp_x)
        # print("     on_x_axis =         ",on_x_axis)

        if( (action==2 or action ==3 or action==6 or action==7) and UR5Env.p_y < UR5Env.lp_y  and UR5Env.p_y > -0.0002):
            on_y_axis = 120*(0.01-UR5Env.p_y)
        else:
            on_y_axis=-120*(abs(0.01-UR5Env.p_y))
        UR5Env.lp_y= UR5Env.p_y
        # print("=?UR5Env.lp_y===  ", UR5Env.lp_y)
        # print("     on_y_axis =         ",on_y_axis)

        if( (action==4 or action ==5 or action==6 or action==7) and UR5Env.p_z > UR5Env.lp_z and UR5Env.p_z < 0.542 ):
            on_z_axis = 1/((0.535 % UR5Env.p_z)+1)
        else:
            on_z_axis = -1/((0.5437 % UR5Env.p_z)+1)
        UR5Env.lp_z = UR5Env.p_z
        # print("=?UR5Env.lp_z===  ", UR5Env.lp_z)
        # print("     on_z_axis =         ",on_z_axis)

        UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
        UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
        UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
        UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
        UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
        print("-"*30)
        print(UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg)
        print("-"*30)

        collision_detection=True
        # print("UR5Env.f_x_avg",UR5Env.f_x_avg)
        # print("UR5Env.f_y_avg",UR5Env.f_y_avg)
        # print("UR5Env.f_z_avg",UR5Env.f_z_avg)
        # print("UR5Env.t_x_avg",UR5Env.t_x_avg)
        # print("UR5Env.t_y_avg",UR5Env.t_y_avg)
        # print("UR5Env.t_z_avg",UR5Env.t_z_avg)
        touch_detection=False
        touch=0
        collision=1
        frame_touch=0
        if( UR5Env.f_x_avg > -15 and UR5Env.f_x_avg < 15 and UR5Env.f_y_avg > 1 and UR5Env.f_y_avg < 36 and UR5Env.f_z_avg > -15 and UR5Env.f_z_avg < 15 and UR5Env.t_x_avg > -15 and UR5Env.t_x_avg < 15 and UR5Env.t_y_avg > -15 and UR5Env.t_y_avg < 15 and UR5Env.t_z_avg < 15 and UR5Env.t_z_avg > -15 ):
            collision_detection = False
            collision=0
            touch=0
            if( UR5Env.f_x_avg < -2 or UR5Env.f_x_avg > 2 or UR5Env.f_y_avg > 17 or UR5Env.f_y_avg < 15 or UR5Env.f_z_avg <- 2 or UR5Env.f_x_avg > 2 or UR5Env.t_x_avg < -2 or UR5Env.t_x_avg > 2 or UR5Env.t_y_avg < -2 or UR5Env.t_y_avg > 2 or UR5Env.t_z_avg > 2 or UR5Env.t_z_avg < -2 ):
                touch_detection = True
                touch=1
                pose_targe = geometry_msgs.msg.Pose()
                pose_targe.orientation.w = -0.5
                pose_targe.orientation.x = 0.5
                pose_targe.orientation.y = -0.5
                pose_targe.orientation.z = 0.5
                if( touch_detection ):
                    pose_targe.position.x = UR5Env.p_x
                    pose_targe.position.y = UR5Env.p_y+0.0001
                    pose_targe.position.z = UR5Env.p_z
                    self.arm_group.set_pose_target(pose_targe)
                    plan1=self.arm_group.go()
                    if( UR5Env.f_x_avg > 1.5 and UR5Env.f_y_avg < 15 and UR5Env.f_z_avg < -1.5 and UR5Env.f_x_avg < 8 and UR5Env.f_y_avg > 5 and UR5Env.f_z_avg > -8 ):
        				frame_touch = frame_touch + 1
                if( touch_detection ):
                    pose_targe.position.x = UR5Env.p_x
                    pose_targe.position.y = UR5Env.p_y-0.0001
                    pose_targe.position.z = UR5Env.p_z
                    self.arm_group.set_pose_target(pose_targe)
                    plan1 = self.arm_group.go()
                    UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg = self.ft_sensors_listener_filter()
                    if(UR5Env.f_x_avg < -1.5 and UR5Env.f_y_avg > 12 and UR5Env.f_z_avg < -1.5 and UR5Env.f_x_avg > -8 and UR5Env.f_y_avg < 16 and UR5Env.f_z_avg > -8):
                    						frame_touch = frame_touch+1
                if(touch_detection):
                    pose_targe.position.x = UR5Env.p_x
                    pose_targe.position.y = UR5Env.p_y
                    pose_targe.position.z = UR5Env.p_z+0.0001
                    self.arm_group.set_pose_target(pose_targe)
                    plan1 = self.arm_group.go()
                    UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
                    if(UR5Env.f_y_avg > 17 and UR5Env.f_z_avg < -1.6 and UR5Env.f_x_avg < -1.5):
                        frame_touch = frame_touch + 1
                if(touch_detection):
                    pose_targe.position.x = UR5Env.p_x
                    pose_targe.position.y = UR5Env.p_y
                    pose_targe.position.z = UR5Env.p_z-0.0001
                    self.arm_group.set_pose_target(pose_targe)
                    plan1 = self.arm_group.go()
                    UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg = self.ft_sensors_listener_filter()
                    if(UR5Env.f_y_avg < 15 and UR5Env.t_x_avg > 1.5):
                        frame_touch = frame_touch + 1
        # print(collision_detection,"="*30)
        return on_x_axis, on_y_axis, on_z_axis, collision_detection, touch_detection, frame_touch, collision, touch

    def ft_sensors_listener_filter(self):
        # print("ft_sensors_listener_filter")
        msg = rospy.wait_for_message("/ft_sensor/raw",WrenchStamped)
        # rospy.loginfo("/ft_sensor/raw")
        # print(msg.wrench.force)
        # print(msg.wrench.torque)
        UR5Env.f_x = msg.wrench.force.x
        UR5Env.f_y = msg.wrench.force.y
        UR5Env.f_z = msg.wrench.force.z

        UR5Env.t_x = msg.wrench.torque.x
        UR5Env.t_y = msg.wrench.torque.y
        UR5Env.t_z = msg.wrench.torque.z
        if(np.isnan(UR5Env.f_x)):
            rospy.wait_for_service('/gazebo/pause_physics')
            try:
                self.pause()
            except (rospy.ServiceException) as e:
                print ("/gazebo/pause_physics service call failed")

        UR5Env.l_fx.append(UR5Env.f_x)
        UR5Env.l_fy.append(UR5Env.f_y)
        UR5Env.l_fz.append(UR5Env.f_z)
        UR5Env.l_tx.append(UR5Env.t_x)
        UR5Env.l_ty.append(UR5Env.t_y)
        UR5Env.l_tz.append(UR5Env.t_z)

        UR5Env.f_x_avg = (sum(UR5Env.l_fx))/3
        UR5Env.f_y_avg = (sum(UR5Env.l_fy))/3
        UR5Env.f_z_avg = (sum(UR5Env.l_fz))/3
        UR5Env.t_x_avg = ( sum(UR5Env.l_tx))/3
        UR5Env.t_y_avg = (sum(UR5Env.l_ty))/3
        UR5Env.t_z_avg = (sum(UR5Env.l_tz))/3

        # print("UR5Env.f_x_avg",UR5Env.f_x_avg)
        # print("UR5Env.f_y_avg",UR5Env.f_y_avg)
        # print("UR5Env.f_z_avg",UR5Env.f_z_avg)
        # print("UR5Env.t_x_avg",UR5Env.t_x_avg)
        # print("UR5Env.t_y_avg",UR5Env.t_y_avg)
        # print("UR5Env.t_z_avg",UR5Env.t_z_avg)
        return UR5Env.t_x_avg, UR5Env.t_y_avg, UR5Env.t_z_avg, UR5Env.f_x_avg, UR5Env.f_y_avg, UR5Env.f_z_avg

    def _seed(self,seed=None):
        self.np_random,seed = seeding.np_random(seed)
        return [seed]

    def step(self,action):
        
        print("="*30)
        print("action = ", action)
        print("old coordinate = ",UR5Env.p_x,UR5Env.p_y,UR5Env.p_z)


        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physicsservicecallfailed")

        pose_targe = geometry_msgs.msg.Pose()
        pose_targe.orientation.w = -0.5
        pose_targe.orientation.x = 0.5
        pose_targe.orientation.y = -0.5
        pose_targe.orientation.z = 0.5
        x,y,z=0,0,0
        step_size=0.01
        self.arm_group.set_max_velocity_scaling_factor(1)
        self.arm_group.set_max_acceleration_scaling_factor(1)
        if(UR5Env.p_x> 0.635 and UR5Env.p_y < 0.01 and UR5Env.p_y > -0.01 and UR5Env.p_z <0.55 and UR5Env.p_z > 0.53):
            step_size=0.000005
            self.arm_group.set_max_velocity_scaling_factor(0.5)
            self.arm_group.set_max_acceleration_scaling_factor(0.01)

        if (action == 0):#Movingonxaxis
            UR5Env.p_x = UR5Env.p_x + step_size
            x=1
            pose_targe.position.x = UR5Env.p_x
            pose_targe.position.y = UR5Env.p_y
            pose_targe.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_targe)
            plan1 = self.arm_group.go()
            # print("f_x:=",UR5Env.f_x)
            # print("f_y:=",UR5Env.f_y)
            # print("f_z:=",UR5Env.f_z)
            # print("p_x",UR5Env.p_x)

        elif (action == 1):#Movingonxaxis

            UR5Env.p_x = UR5Env.p_x - step_size
            x=-1
            pose_targe.position.x = UR5Env.p_x
            pose_targe.position.y = UR5Env.p_y
            pose_targe.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_targe)
            plan1 = self.arm_group.go()

        	# print("f_x:=",UR5Env.f_x)
        	# print("f_y:=",UR5Env.f_y)
        	# print("f_z:=",UR5Env.f_z)
        	# print("p_x",UR5Env.p_x)


        elif (action == 2):##Movingonyaxis

            UR5Env.p_y = UR5Env.p_y + step_size
            y=1
            pose_targe.position.x = UR5Env.p_x
            pose_targe.position.y = UR5Env.p_y
            pose_targe.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_targe)
            plan1 = self.arm_group.go()
            # print("f_x:=",UR5Env.f_x)
            # print("f_y:=",UR5Env.f_y)
            # print("f_z:=",UR5Env.f_z)
            print("p_z",UR5Env.p_y)

        elif (action == 3):##Movingonyaxis

            UR5Env.p_y = UR5Env.p_y - step_size
            y=-1
            pose_targe.position.x = UR5Env.p_x
            pose_targe.position.y = UR5Env.p_y
            pose_targe.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_targe)
            plan1 = self.arm_group.go()
            # print("f_x:=",UR5Env.f_x)
            # print("f_y:=",UR5Env.f_y)
            # print("f_z:=",UR5Env.f_z)
            # print("p_y",UR5Env.p_y)

        elif (action == 4):##Movingonzaxis

            UR5Env.p_z = UR5Env.p_z + step_size
            z=1
            pose_targe.position.x = UR5Env.p_x
            pose_targe.position.y = UR5Env.p_y
            pose_targe.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_targe)
            plan1 = self.arm_group.go()
            # print("f_x:=",UR5Env.f_x)
            # print("f_y:=",UR5Env.f_y)
            # print("f_z:=",UR5Env.f_z)
            # print("p_z",UR5Env.p_z)


        elif (action == 5):#Movingonzaxis
            UR5Env.p_z = UR5Env.p_z - step_size
            z=-1
            pose_targe.position.x = UR5Env.p_x
            pose_targe.position.y = UR5Env.p_y
            pose_targe.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_targe)
            plan1 = self.arm_group.go()
            # print("f_x:=",UR5Env.f_x)
            # print("f_y:=",UR5Env.f_y)
            # print("f_z:=",UR5Env.f_z)
            # print("p_z",UR5Env.p_z)

        elif (action == 6):
            UR5Env.p_x = UR5Env.p_x + step_size
            UR5Env.p_y = UR5Env.p_y + step_size
            UR5Env.p_z = UR5Env.p_z + step_size
            x=1
            y=-1
            z=1
            pose_targe.position.x = UR5Env.p_x
            pose_targe.position.y = UR5Env.p_y
            pose_targe.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_targe)
            plan1 = self.arm_group.go()

        elif (action == 7):
            UR5Env.p_x = UR5Env.p_x + step_size
            UR5Env.p_y = UR5Env.p_y - step_size
            UR5Env.p_z = UR5Env.p_z + step_size
            x=1
            y=-1
            z=1
            pose_targe.position.x = UR5Env.p_x
            pose_targe.position.y = UR5Env.p_y
            pose_targe.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_targe)
            plan1 = self.arm_group.go()



        print("new coordinate = ",UR5Env.p_x,UR5Env.p_y,UR5Env.p_z)


        # elif (action == 6):#Movingonzaxis
        #     pose_targe.position.x = UR5Env.p_x
        #     pose_targe.position.y = UR5Env.p_y
        #     pose_targe.position.z = UR5Env.p_z
        #     self.arm_group.set_pose_target(pose_targe)
        #     plan1 = self.arm_group.go()
        #     print("f_x:=",UR5Env.f_x)
        #     print("f_y:=",UR5Env.f_y)
        #     print("f_z:=",UR5Env.f_z)
        #     print("stop")



        x_axis,y_axis,z_axis,Collision,touch,frame_int,collision_1,touch_1 = self.state_observation(action)


        reward = 0
        done = True
        #state = [UR5Env.p_x,UR5Env.p_y,UR5Env.p_z]
        state = [x,y,z,collision_1,touch_1]
        print("state",state )
        done = Collision

        if(action==0 or action==1 or action==6 or action==7 ):
            reward = reward + x_axis
        if(action==2 or action==3 or action==6 or action==7):
            reward = reward + y_axis
        if( action == 4 or action == 5 or action==6 or action==7):
            reward = reward + z_axis
        if(touch):
        	reward = reward + 11
        if(frame_int > 0):
        	reward = reward + 100
        if(frame_int > 1):
        	reward = reward + 200
        if(frame_int > 2):
        	reward = reward + 1000
        if(frame_int > 2 and touch and Collision and x_axis and y_axis and z_axis):
        	done=True
        # print("iter,UR5Env.p_x, UR5Env.p_z, UR5Env.p_z", iter,UR5Env.p_x, UR5Env.p_z, UR5Env.p_z)
        if(Collision):
            reward=reward-100


        print(" reward = ",reward)
        print("="*30)
        if(UR5Env.p_x > 0.7 or UR5Env.p_x > 0.2 and UR5Env.p_z > 0.7 and UR5Env.p_z < 0.3 or UR5Env.p_y < -0.2 and UR5Env.p_y < 0.2 ):
            done=True

        return state, reward, done, {}


    def reset(self):
        UR5Env.p_x,UR5Env.p_y,UR5Env.p_z = 0.5, 0.1, 0.435
        UR5Env.lp_x,UR5Env.lp_y,UR5Env.lp_z = 0.5, 0.1, 0.435
    	self.arm_group.set_max_velocity_scaling_factor(1)
    	self.arm_group.set_max_acceleration_scaling_factor(1)
    	pose_targe = geometry_msgs.msg.Pose()
    	pose_targe.orientation.w = -0.5
    	pose_targe.orientation.x = 0.5
    	pose_targe.orientation.y = -0.5
    	pose_targe.orientation.z = 0.5
    	pose_targe.position.x = 0.4
    	pose_targe.position.y = 0.0
    	pose_targe.position.z = 0.5
    	self.arm_group.set_pose_target(pose_targe)
    	plan1 = self.arm_group.go()
    	pose_targe.position.x = 0.5111
    	pose_targe.position.y = 0.3
    	pose_targe.position.z = 0.45
    	self.arm_group.set_pose_target(pose_targe)
    	plan1 = self.arm_group.go()
    	self.arm_group.set_max_velocity_scaling_factor(0.1)
    	self.arm_group.set_max_acceleration_scaling_factor(0.1)


        return[ 0, 0, 0,0,0]
