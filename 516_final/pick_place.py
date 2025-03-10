#!/usr/bin/env python3
import time
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from pyquaternion import Quaternion as PyQuaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper  
import numpy as np

def pick_and_place(arm, gripper, pre_pick_pose, pick_pose, pre_put_pose1, pre_put_pose2, put_pose1, put_pose2):
    arm.inverse_kinematic_movement(pre_pick_pose)
    time.sleep(0.5)
    gripper.move_to_position(0.0)
    arm.inverse_kinematic_movement(pick_pose)
    gripper.move_to_position(0.5)
    print("got the cube")
    time.sleep(0.5)
    arm.inverse_kinematic_movement(pre_put_pose1)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(pre_put_pose2)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(put_pose1)
    time.sleep(0.5)
    arm.inverse_kinematic_movement(put_pose2)
    time.sleep(0.5)
    gripper.move_to_position(0.0)
    print("finished!")

def set_pose(position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w):
    return_pose = Pose()
    return_pose.position.x = position_x
    return_pose.position.y = position_y
    return_pose.position.z = position_z
    return_pose.orientation.x = orientation_x
    return_pose.orientation.y = orientation_y
    return_pose.orientation.z = orientation_z
    return_pose.orientation.w = orientation_w
    return return_pose

def main():
    rclpy.init()
    # Initialize arm and gripper interfaces
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()
    pre_pick_pose = set_pose(0.380, -0.030, 0.371, 0.817, 0.574, -0.036, 0.028)
    pick_pose = set_pose(0.394, -0.031, 0.161, 0.751, 0.659, -0.025, 0.017)
    pre_put_pose1 = set_pose(-.293, 0.081, 0.228, 0.752, 0.658, 0.023, -0.027)
    pre_put_pose2 = set_pose(-0.174, -0.002, 0.191, 0.494, 0.867, -0.007, -0.066)
    put_pose1 = set_pose(-0.195, 0.034, -0.164, 0.494, 0.867, -0.008, -0.066)
    put_pose2 = set_pose(-0.173, 0.0, -0.342, 0.542, 0.838, -0.04, -0.042)
    pick_and_place(arm, gripper, pre_pick_pose, pick_pose, pre_put_pose1, pre_put_pose2, put_pose1, put_pose2)
    # Shutdown ROS2 and clean up
    rclpy.shutdown()
    gripper.shutdown()
    arm.shutdown()

if __name__ == '__main__':
    main()


