#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatusArray

import moveit_commander
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import tf
import math
import time
import yaml

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal


PI = math.pi

def set_pose(x, y, z, rz, ry, rx):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    x,y,z,w = tf.transformations.quaternion_from_euler(rz, ry, rx)
    pose.orientation.x = x
    pose.orientation.y = y
    pose.orientation.z = z
    pose.orientation.w = w

    arm.set_start_state_to_current_state()
    arm.set_pose_target(pose)

    print(arm.get_current_pose())

    ret = arm.go()

    return ret

def open_gripper( _open=True ):
    gripper.set_start_state_to_current_state()
    if _open:
        gripper.set_joint_value_target([0.7, 0.7])
    else:
        gripper.set_joint_value_target([0.1, 0.1])
    gripper.go()

def set_init_pose():
    arm.set_joint_value_target( [0, -0.3, 0, -2.5, 0, 1.2, 1.57] )
    arm.go()


def grasp(x, y, z):
    # 初期位置・グリッパを開く
    set_init_pose()
    open_gripper()

    # アーム移動、グリッパを閉じる
    if z<0.1:
        z = 0.1
    theta = math.atan2( y, x )

    # 一旦手前にハンドを持っていく
    set_pose( x-0.1*math.cos(theta), y-0.1*math.sin(theta), z, PI/2, 0,  PI/2+theta  )

    # 物体位置へ移動
    set_pose( x, y, z, PI/2, 0,  PI/2+theta  )
    open_gripper( False )

    # 持ち上げる
    set_pose( x, y, z+0.1, PI/2, 0,  PI/2+theta  )

     # 初期位置
    set_init_pose()


def main():
    set_init_pose()
    #joint_state = actionlib.SimpleActionClient("/crane_x7/arm_controller/follow_joint_trajectory/status", GoalStatusArray)

    joint_state = rospy.wait_for_message("/joint_states", JointState)
    #print("joint_state",joint_state)

    trajectory_client = actionlib.SimpleActionClient("/crane_x7/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    trajectory_client.wait_for_server(timeout=rospy.Duration(60.0))
    print("trajectory_client", trajectory_client)

    joint_names = ["crane_x7_lower_arm_fixed_part_joint"
      ,"crane_x7_lower_arm_revolute_part_joint"
      ,"crane_x7_shoulder_fixed_part_pan_joint"
      ,"crane_x7_shoulder_revolute_part_tilt_joint"
      ,"crane_x7_upper_arm_revolute_part_rotate_joint"
      ,"crane_x7_upper_arm_revolute_part_twist_joint"
      ,"crane_x7_wrist_joint"
]

    point = JointTrajectoryPoint()
    point.time_from_start = rospy.Duration(1.0)
    point.positions = [-0.0, 0.0, -0.0, -0.0, -0.0, 0.0, 0.0]
    point.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    #point.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    #print("crane_traj", point.postions)

    point2 = JointTrajectoryPoint()
    point2.time_from_start = rospy.Duration(5.0)
    point2.positions = [-0.1, 0.1, -0.1, -0.1, -0.1, 0.1, 0.1]
    point2.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    trajectory_goal = FollowJointTrajectoryGoal()
    trajectory_goal.trajectory.joint_names = joint_names
    trajectory_goal.trajectory.points = [point, point2]
    trajectory_goal.trajectory.header.stamp = rospy.Time.now()
    trajectory_client.send_goal(trajectory_goal)

if __name__ == "__main__":
    rospy.init_node("crane")

    tf_listener = tf.TransformListener()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    main()

    rospy.sleep(10)