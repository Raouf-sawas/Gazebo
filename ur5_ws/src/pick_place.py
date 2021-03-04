#!/usr/bin/env python33

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
    goal.command.max_effort = 6.0  # Do not limit the effort
    print(goal.command.max_effort)
    client.send_goal(goal)

    client.wait_for_result()
    return client.get_result()


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('moveit_group_python_interface',anonymous=True)
robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("arm")
arm_group.set_max_velocity_scaling_factor(1)
arm_group.set_max_acceleration_scaling_factor(0.01)
# Note this is not the speed of the end-effector point.
#   group.setMaxVelocityScalingFactor(0.005);
#   group.setMaxAccelerationScalingFactor(0.001);



pose_targe = geometry_msgs.msg.Pose()


pose_targe = geometry_msgs.msg.Pose()
pose_targe.orientation.w = -0.5
pose_targe.orientation.x = 0.5
pose_targe.orientation.y = -0.5
pose_targe.orientation.z = 0.5
#pose_targe.position.x = 0.4
pose_targe.position.x = 0.42111
pose_targe.position.y = 0.0
pose_targe.position.z = 0.515018
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()

print("first")
gripper_client(0.0)


pose_targe.position.x = 0.4218
pose_targe.position.y = 0.0
pose_targe.position.z = 0.5
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()



print("sec")
pose_targe.position.x = 0.4218
pose_targe.position.y = 0.0
pose_targe.position.z = 0.523
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()

#pick done
#arm_group.set_max_acceleration_scaling_factor(0.001)

rospy.sleep(1)


pose_targe.position.x = 0.43
pose_targe.position.y = 0.0
pose_targe.position.z = 0.522
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()
print("thirfd")
arm_group.set_max_acceleration_scaling_factor(0.2)
rospy.sleep(2)
pose_targe.position.x = 0.43
pose_targe.position.y = 0.0
pose_targe.position.z = 0.522
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()

pose_targe.position.x = 0.46
pose_targe.position.y = 0.0
pose_targe.position.z = 0.522
arm_group.set_pose_target(pose_targe)
plan1 =arm_group.go()


print("fourth")
moveit_commander.roscpp_shutdown()
