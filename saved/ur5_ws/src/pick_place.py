#!/usr/bin/env python

import sys
import rospy
import moveit_commander

import geometry_msgs.msg

import random
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
import argparse
import actionlib
import control_msgs.msg




def gripper_client(value):

    # Create an action client
    client = actionlib.SimpleActionClient(
        '/gripper_controller/gripper_cmd',  # namespace of the action topics
        control_msgs.msg.GripperCommandAction # action type
    )
    
    # Wait until the action server has been started and is listening for goals
    client.wait_for_server()

    # Create a goal to send (to the action server)
    goal = control_msgs.msg.GripperCommandGoal()
    #goal.command.position =  random.uniform(0.0,0.8)    # From 0.0 to 0.8
    goal.command.position =  value
    goal.command.max_effort = 1.0  # Do not limit the effort
    print(goal.command.max_effort)
    client.send_goal(goal)

    client.wait_for_result()
    return client.get_result()    


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_group_python_interface',anonymous=True)
robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("arm")



pose_targe = geometry_msgs.msg.Pose()


pose_targe = geometry_msgs.msg.Pose()
pose_targe.orientation.w = -0.5
pose_targe.orientation.x = 0.5
pose_targe.orientation.y = -0.5
pose_targe.orientation.z = 0.5
#pose_targe.position.x = 0.4
pose_targe.position.x = 0.2
pose_targe.position.y = 0.3
pose_targe.position.z = 0.64
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()






pose_targe.orientation.w = -0.5
pose_targe.orientation.x = 0.5
pose_targe.orientation.y = -0.5
pose_targe.orientation.z = 0.5
pose_targe.position.x = 0.618
pose_targe.position.y = 0.3
pose_targe.position.z = 0.48
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()





pose_targe = geometry_msgs.msg.Pose()
pose_targe.orientation.w = -0.5
pose_targe.orientation.x = 0.5
pose_targe.orientation.y = -0.5
pose_targe.orientation.z = 0.5
pose_targe.position.x = 0.618
pose_targe.position.y = 0.3
pose_targe.position.z = 0.435
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()


rospy.sleep(3)
gripper_client(0.1)
#pick done

rospy.sleep(5)


pose_targe = geometry_msgs.msg.Pose()
pose_targe.orientation.w = -0.5
pose_targe.orientation.x = 0.5
pose_targe.orientation.y = -0.5
pose_targe.orientation.z = 0.5
#pose_targe.position.x = 0.4
pose_targe.position.x = 0.2
pose_targe.position.y = 0.3
pose_targe.position.z = 0.64
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()


rospy.sleep(1)

pose_targe = geometry_msgs.msg.Pose()
pose_targe.orientation.w = -0.5
pose_targe.orientation.x = 0.5
pose_targe.orientation.y = -0.5
pose_targe.orientation.z = 0.5
#pose_targe.position.x = 0.4
pose_targe.position.x = 0.2
pose_targe.position.y = 0
pose_targe.position.z = 0.63
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()


rospy.sleep(1)




pose_targe = geometry_msgs.msg.Pose()
pose_targe.orientation.w = -0.5
pose_targe.orientation.x = 0.5
pose_targe.orientation.y = -0.5
pose_targe.orientation.z = 0.5
#pose_targe.position.x = 0.4
pose_targe.position.x = 0.35
pose_targe.position.y = 0
pose_targe.position.z = 0.63
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()


rospy.sleep(2)


pose_targe = geometry_msgs.msg.Pose()
pose_targe.orientation.w = -0.5
pose_targe.orientation.x = 0.5
pose_targe.orientation.y = -0.5
pose_targe.orientation.z = 0.5
#pose_targe.position.x = 0.4
pose_targe.position.x = 0.45
pose_targe.position.y = 0
pose_targe.position.z = 0.635
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()


rospy.sleep(2)

gripper_client(0.0)
rospy.sleep(3)
pose_targe = geometry_msgs.msg.Pose()
pose_targe.orientation.w = -0.5
pose_targe.orientation.x = 0.5
pose_targe.orientation.y = -0.5
pose_targe.orientation.z = 0.5
#pose_targe.position.x = 0.4
pose_targe.position.x = 0.15
pose_targe.position.y = 0
pose_targe.position.z = 0.635
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()


moveit_commander.roscpp_shutdown()






 


