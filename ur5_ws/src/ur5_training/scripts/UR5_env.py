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
import math
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
    step_size=0
    n_steps=0
    p_x,lp_x, lp_y, p_y,lp_z, p_z = 0.6, 0.6, 0.26, 0.26, 0.435, 0.435
    x_goal,y_goal,z_goal=0.6611 ,0.318, 0.54372018
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_group_python_interface',anonymous=True)
        self.unpause=rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        self.pause=rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        self.reset_proxy=rospy.ServiceProxy('/gazebo/reset_simulation',Empty)
        self.robot=moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.p = geometry_msgs.msg.PoseStamped()
        self.p.header.frame_id = "table"
        self.p.header.frame_id = self.robot.get_planning_frame()
        self.p.pose.position.x = 0.788
        self.p.pose.position.y = 0.6
        self.p.pose.position.z =  0.433
        self.scene.add_box("table", self.p, (1, 2, 1))

        self.p = geometry_msgs.msg.PoseStamped()
        self.p.header.frame_id = "robot_pilar"
        self.p.header.frame_id = self.robot.get_planning_frame()
        self.p.pose.position.x = 0.0
        self.p.pose.position.y = 0.0
        self.p.pose.position.z = 0.333
        self.scene.add_box("robot_pilar", self.p, (0.1, 0.01, 0.5))



        self.arm_group=moveit_commander.MoveGroupCommander("arm")
        self.action_space=spaces.Discrete(6)
        self.reward_range=(-np.inf,np.inf)
        self._seed()

    def gripper_client(self,value):
        client = actionlib.SimpleActionClient(
            '/gripper_controller/gripper_cmd',
                control_msgs.msg.GripperCommandAction)
        client.wait_for_server()
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position =  value
        goal.command.max_effort = 6.0
        client.send_goal(goal)
        client.wait_for_result()

    def pick(self):
        pose_target = geometry_msgs.msg.Pose()
        self.arm_group.set_max_velocity_scaling_factor(1)
        self.arm_group.set_max_acceleration_scaling_factor(0.01)
        pose_target.orientation.w = -0.5
        pose_target.orientation.x = 0.5
        pose_target.orientation.y = -0.5
        pose_target.orientation.z = 0.5
        pose_target.position.x = -0.553
        pose_target.position.y = 0.3
        pose_target.position.z = 0.5
        self.arm_group.set_pose_target(pose_target)
        plan1 = self.arm_group.go()

        self.gripper_client(0.0)
        self.arm_group.set_pose_target(pose_target)
        plan1 = self.arm_group.go()
        pose_target.position.x = -0.63
        pose_target.position.y = 0.3
        pose_target.position.z = 0.433
        self.arm_group.set_pose_target(pose_target)
        plan1 = self.arm_group.go()

        self.gripper_client(0.46)
        rospy.sleep(10)

        pose_target.position.x = 0.5
        pose_target.position.y = 0.2
        pose_target.position.z = 0.533
        self.arm_group.set_pose_target(pose_target)
        plan1 =self.arm_group.go()

    def state_observation(self, action):
        on_x_axis, on_y_axis, on_z_axis = 0,0,0

        if( (action==0 or action ==1 or action==6 or action==7) ):
            if UR5Env.p_x > UR5Env.lp_x  or (UR5Env.p_x < 0.712 and  UR5Env.p_x > 6.1)  :
            	on_x_axis = 1
            else:
                on_x_axis = -1
        UR5Env.lp_x = UR5Env.p_x

        if( (action==2 or action ==3 or action==6 or action==7)):
            if (UR5Env.p_y > UR5Env.lp_y   and UR5Env.p_y < 0.33 ):
                on_y_axis = 1
            else:
                on_y_axis=-1
        UR5Env.lp_y= UR5Env.p_y
        if( (action==4 or action ==5 or action==6 or action==7) ):
            if UR5Env.p_z > UR5Env.lp_z and UR5Env.p_z < 0.544  :
                on_z_axis = 1
            else:
                on_z_axis = -1
        UR5Env.lp_z = UR5Env.p_z

        UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
        UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
        UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
        UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
        UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
        #observation =[UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg,UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg]
        #observation =[int(UR5Env.f_x_avg, 2),int(UR5Env.f_y_avg, 2),int(UR5Env.f_z_avg, 2),int(UR5Env.t_x_avg, 2),int(UR5Env.t_y_avg, 2),int(UR5Env.t_z_avg, 2)]
        #observation =[round(UR5Env.p_x, 4),round(UR5Env.p_y, 4),round(UR5Env.p_z, 4),int(UR5Env.f_x_avg),int(UR5Env.f_y_avg ),int(UR5Env.f_z_avg ),int(UR5Env.t_x_avg ),int(UR5Env.t_y_avg ),int(UR5Env.t_z_avg )]
        if math.isnan(UR5Env.f_x_avg ):
            observation =[UR5Env.f_x_avg ,UR5Env.f_y_avg ,UR5Env.f_z_avg,UR5Env.t_x_avg,UR5Env.t_y_avg ,UR5Env.t_z_avg]
        else:
            observation =[int(UR5Env.f_x_avg),int(UR5Env.f_y_avg ),int(UR5Env.f_z_avg ),
            int(UR5Env.t_x_avg ),int(UR5Env.t_y_avg ),int(UR5Env.t_z_avg )]


        print("-"*30)
        print(observation)
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
        if( UR5Env.f_x_avg > -30 and UR5Env.f_x_avg < 30 and UR5Env.f_y_avg > -40 and UR5Env.f_y_avg < 40 and UR5Env.f_z_avg > -30 and UR5Env.f_z_avg < 30 and UR5Env.t_x_avg > -30 and UR5Env.t_x_avg < 30 and UR5Env.t_y_avg > -30 and UR5Env.t_y_avg < 30 and UR5Env.t_z_avg < 30 and UR5Env.t_z_avg > -30 ):
            collision_detection = False
            UR5Env.step_size = 0.00001
            collision=0
            touch=0
            if( UR5Env.f_x_avg < -1 or UR5Env.f_x_avg > 2 or UR5Env.f_y_avg > 17 or UR5Env.f_y_avg < 15 or UR5Env.f_z_avg <- 2 or UR5Env.f_x_avg > 2 or UR5Env.t_x_avg < -2 or UR5Env.t_x_avg > 2 or UR5Env.t_y_avg < -2 or UR5Env.t_y_avg > 2 or UR5Env.t_z_avg > 2 or UR5Env.t_z_avg < -2 ):
                touch_detection = True
                touch=1
                pose_target = geometry_msgs.msg.Pose()
                pose_target.orientation.w = -0.5
                pose_target.orientation.x = 0.5
                pose_target.orientation.y = -0.5
                pose_target.orientation.z = 0.5
                if( touch_detection ):
                    pose_target.position.x = UR5Env.p_x
                    pose_target.position.y = UR5Env.p_y+0.0001
                    pose_target.position.z = UR5Env.p_z
                    self.arm_group.set_pose_target(pose_target)
                    plan1=self.arm_group.go()
                    if( UR5Env.f_x_avg > 1.5 and UR5Env.f_y_avg < 15 and UR5Env.f_z_avg < -1.5 and UR5Env.f_x_avg < 8 and UR5Env.f_y_avg > 5 and UR5Env.f_z_avg > -8 ):
        				frame_touch = frame_touch + 1
                if( touch_detection ):
                    pose_target.position.x = UR5Env.p_x
                    pose_target.position.y = UR5Env.p_y-0.0001
                    pose_target.position.z = UR5Env.p_z
                    self.arm_group.set_pose_target(pose_target)
                    plan1 = self.arm_group.go()
                    UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg = self.ft_sensors_listener_filter()
                    if(UR5Env.f_x_avg < -1.5 and UR5Env.f_y_avg > 12 and UR5Env.f_z_avg < -1.5 and UR5Env.f_x_avg > -8 and UR5Env.f_y_avg < 16 and UR5Env.f_z_avg > -8):
                    						frame_touch = frame_touch+1
                if(touch_detection):
                    pose_target.position.x = UR5Env.p_x
                    pose_target.position.y = UR5Env.p_y
                    pose_target.position.z = UR5Env.p_z+0.0001
                    self.arm_group.set_pose_target(pose_target)
                    plan1 = self.arm_group.go()
                    UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
                    if(UR5Env.f_y_avg > 17 and UR5Env.f_z_avg < -1.6 and UR5Env.f_x_avg < -1.5):
                        frame_touch = frame_touch + 1
                if(touch_detection):
                    pose_target.position.x = UR5Env.p_x
                    pose_target.position.y = UR5Env.p_y
                    pose_target.position.z = UR5Env.p_z-0.0001
                    self.arm_group.set_pose_target(pose_target)
                    plan1 = self.arm_group.go()
                    UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg = self.ft_sensors_listener_filter()
                    if(UR5Env.f_y_avg < 15 and UR5Env.t_x_avg > 1.5):
                        frame_touch = frame_touch + 1
        # print(collision_detection,"="*30)
        return observation,on_x_axis, on_y_axis, on_z_axis, collision_detection, touch_detection, frame_touch, collision, touch

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
        # if(np.isnan(UR5Env.f_x)):
        #     self.ft_sensors_listener_filter()
            # rospy.wait_for_service('/gazebo/pause_physics')
            # try:
            #     self.pause()
            # except (rospy.ServiceException) as e:
            #     print ("/gazebo/pause_physics service call failed")

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

        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = -0.5
        pose_target.orientation.x = 0.5
        pose_target.orientation.y = -0.5
        pose_target.orientation.z = 0.5
        x,y,z=0,0,0
        UR5Env.step_size=0.0001
        reward = 0
        self.arm_group.set_max_velocity_scaling_factor(1)
        self.arm_group.set_max_acceleration_scaling_factor(1)
        if(UR5Env.p_x> 0.608 and UR5Env.p_y < 0.36 and UR5Env.p_y > 0.27 and UR5Env.p_z <0.57 and UR5Env.p_z > 0.51):
            UR5Env.step_size=0.00002
            reward=reward+3
            self.arm_group.set_max_velocity_scaling_factor(0.5)
            self.arm_group.set_max_acceleration_scaling_factor(0.01)



        if (action == 0):#Movingonxaxis
            UR5Env.p_x = UR5Env.p_x + UR5Env.step_size
            x=1
            pose_target.position.x = UR5Env.p_x
            pose_target.position.y = UR5Env.p_y
            pose_target.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_target)
            plan1 = self.arm_group.go()
            # print("f_x:=",UR5Env.f_x)
            # print("f_y:=",UR5Env.f_y)
            # print("f_z:=",UR5Env.f_z)
            # print("p_x",UR5Env.p_x)

        elif (action == 1):#Movingonxaxis

            UR5Env.p_x = UR5Env.p_x - UR5Env.step_size
            x=-1
            pose_target.position.x = UR5Env.p_x
            pose_target.position.y = UR5Env.p_y
            pose_target.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_target)
            plan1 = self.arm_group.go()

        	# print("f_x:=",UR5Env.f_x)
        	# print("f_y:=",UR5Env.f_y)
        	# print("f_z:=",UR5Env.f_z)
        	# print("p_x",UR5Env.p_x)


        elif (action == 2):##Movingonyaxis

            UR5Env.p_y = UR5Env.p_y + UR5Env.step_size
            y=1
            pose_target.position.x = UR5Env.p_x
            pose_target.position.y = UR5Env.p_y
            pose_target.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_target)
            plan1 = self.arm_group.go()
            # print("f_x:=",UR5Env.f_x)
            # print("f_y:=",UR5Env.f_y)
            # print("f_z:=",UR5Env.f_z)
            print("p_z",UR5Env.p_y)

        elif (action == 3):##Movingonyaxis

            UR5Env.p_y = UR5Env.p_y - UR5Env.step_size
            y=-1
            pose_target.position.x = UR5Env.p_x
            pose_target.position.y = UR5Env.p_y
            pose_target.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_target)
            plan1 = self.arm_group.go()
            # print("f_x:=",UR5Env.f_x)
            # print("f_y:=",UR5Env.f_y)
            # print("f_z:=",UR5Env.f_z)
            # print("p_y",UR5Env.p_y)

        elif (action == 4):##Movingonzaxis

            UR5Env.p_z = UR5Env.p_z + UR5Env.step_size
            z=1
            pose_target.position.x = UR5Env.p_x
            pose_target.position.y = UR5Env.p_y
            pose_target.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_target)
            plan1 = self.arm_group.go()
            # print("f_x:=",UR5Env.f_x)
            # print("f_y:=",UR5Env.f_y)
            # print("f_z:=",UR5Env.f_z)
            # print("p_z",UR5Env.p_z)


        elif (action == 5):#Movingonzaxis
            UR5Env.p_z = UR5Env.p_z - UR5Env.step_size
            z=-1
            pose_target.position.x = UR5Env.p_x
            pose_target.position.y = UR5Env.p_y
            pose_target.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_target)
            plan1 = self.arm_group.go()
            # print("f_x:=",UR5Env.f_x)
            # print("f_y:=",UR5Env.f_y)
            # print("f_z:=",UR5Env.f_z)
            # print("p_z",UR5Env.p_z)

        elif (action == 6):
            UR5Env.p_x = UR5Env.p_x + UR5Env.step_size
            UR5Env.p_y = UR5Env.p_y + UR5Env.step_size
            UR5Env.p_z = UR5Env.p_z + UR5Env.step_size
            x=1
            y=1
            z=1
            pose_target.position.x = UR5Env.p_x
            pose_target.position.y = UR5Env.p_y
            pose_target.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_target)
            plan1 = self.arm_group.go()

        elif (action == 7):
            UR5Env.p_x = UR5Env.p_x + UR5Env.step_size
            UR5Env.p_y = UR5Env.p_y - UR5Env.step_size
            UR5Env.p_z = UR5Env.p_z + UR5Env.step_size
            x=1
            y=-1
            z=1
            pose_target.position.x = UR5Env.p_x
            pose_target.position.y = UR5Env.p_y
            pose_target.position.z = UR5Env.p_z
            self.arm_group.set_pose_target(pose_target)
            plan1 = self.arm_group.go()



        print("new coordinate = ",UR5Env.p_x,UR5Env.p_y,UR5Env.p_z)


        # elif (action == 6):#Movingonzaxis
        #     pose_target.position.x = UR5Env.p_x
        #     pose_target.position.y = UR5Env.p_y
        #     pose_target.position.z = UR5Env.p_z
        #     self.arm_group.set_pose_target(pose_target)
        #     plan1 = self.arm_group.go()
        #     print("f_x:=",UR5Env.f_x)
        #     print("f_y:=",UR5Env.f_y)
        #     print("f_z:=",UR5Env.f_z)
        #     print("stop")



        observation,x_axis,y_axis,z_axis,Collision,touch,frame_int,collision_1,touch_1 = self.state_observation(action)



        done = True
        print("observation",observation)
        #state = [UR5Env.p_x,UR5Env.p_y,UR5Env.p_z]
        state = [x,y,z,collision_1,touch_1]
        print("state",state )
        done = Collision
        reward = reward + x_axis+y_axis+z_axis


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
            reward=reward-10

        UR5Env.n_steps=UR5Env.n_steps+1


        print(" reward = ",reward)
        print("="*30)
        if(UR5Env.n_steps > 1195 or UR5Env.p_x> 0.75 or UR5Env.p_x < 0.3 or UR5Env.p_y > 0.4 or UR5Env.p_y < -0.3 or UR5Env.p_z <0.35 or UR5Env.p_z > 0.7 or (UR5Env.p_x> 0.65 and UR5Env.p_z> 0.6) ):
            reward= reward - 100
            UR5Env.n_steps=0
            done=True

        return observation, reward, done, {}


    def reset(self):
        UR5Env.p_x,UR5Env.p_y,UR5Env.p_z = 0.6, 0.3, 0.53
        UR5Env.lp_x,UR5Env.lp_y,UR5Env.lp_z = 0.6, 0.3, 0.53
    	self.arm_group.set_max_velocity_scaling_factor(1)
    	self.arm_group.set_max_acceleration_scaling_factor(1)
    	pose_target = geometry_msgs.msg.Pose()
    	pose_target.orientation.w = -0.5
    	pose_target.orientation.x = 0.5
    	pose_target.orientation.y = -0.5
    	pose_target.orientation.z = 0.5
    	pose_target.position.x = 0.5
    	pose_target.position.y = 0.3
    	pose_target.position.z = 0.5
    	self.arm_group.set_pose_target(pose_target)
    	plan1 = self.arm_group.go()
    	pose_target.position.x = 0.59111
    	pose_target.position.y = 0.3
    	pose_target.position.z = 0.52
    	self.arm_group.set_pose_target(pose_target)
    	plan1 = self.arm_group.go()
    	self.arm_group.set_max_velocity_scaling_factor(0.1)
    	self.arm_group.set_max_acceleration_scaling_factor(0.1)
        UR5Env.t_x_avg,UR5Env.t_y_avg,UR5Env.t_z_avg,UR5Env.f_x_avg,UR5Env.f_y_avg,UR5Env.f_z_avg=self.ft_sensors_listener_filter()
        if(UR5Env.f_y_avg < 12 and int(UR5Env.f_x_avg)==0 and int(UR5Env.f_z_avg)==0):
            print("picking")
            reset_simulation = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_simulation()


            # Unpause simulation to make observation

            self.pick()

            # rospy.wait_for_service('/gazebo/reset_world')
            # reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            # reset_world()

        #observation =[round(UR5Env.p_x, 4),round(UR5Env.p_y, 4),round(UR5Env.p_z, 4),int(UR5Env.f_x_avg ),int(UR5Env.f_y_avg ),int(UR5Env.f_z_avg ),int(UR5Env.t_x_avg ),int(UR5Env.t_y_avg ),int(UR5Env.t_z_avg )]
        if math.isnan(UR5Env.f_x_avg ):
            observation =[UR5Env.f_x_avg ,UR5Env.f_y_avg ,UR5Env.f_z_avg,UR5Env.t_x_avg,UR5Env.t_y_avg ,UR5Env.t_z_avg]
        else:
            observation =[int(UR5Env.f_x_avg ),int(UR5Env.f_y_avg ),int(UR5Env.f_z_avg ),int(UR5Env.t_x_avg ),int(UR5Env.t_y_avg ),int(UR5Env.t_z_avg )]


        return observation
