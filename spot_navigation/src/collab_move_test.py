#!/usr/bin/env python3
# coding=utf8

import time
import math
import numpy as np
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

threshold = 0.5
gazebo_amcl_offset = [1.1087, 0.2397] # gazebo_pose + offset = acml_pose (for mymap2.yaml)

spot_pose = None
def spot_amcl_pose_callback(spot_msg):
    global spot_pose
    spot_pose = spot_msg.pose.pose

spot_amcl_goal1 = [2.15, 1.376, np.pi] # [x, y, heading]
spot_amcl_goal2 = [0.15, -0.901, 0.0]
spot_gazebo_goal1 = [1.223, 1.176, np.pi]
spot_gazebo_goal2 = [-1.110, -1.0914, 0.0]

# print(spot_amcl_goal1[0] - spot_gazebo_goal1[0], spot_amcl_goal1[1] - spot_gazebo_goal1[1])
# print(spot_amcl_goal2[0] - spot_gazebo_goal2[0], spot_amcl_goal2[1] - spot_gazebo_goal2[1])
# exit()

def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.
    
    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.
    
    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qx, qy, qz, qw]

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

def dist_check(pose, goal):
    pose_heading = euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2]
    return ((pose.position.x - goal[0])**2 + (pose.position.y - goal[1])**2) ** 0.5, abs(pose_heading - goal[2]) #TODO: wrap2pi

def get_gazebo_pose(model_name):
    rospy.wait_for_service ('/gazebo/get_model_state')
    get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    model = GetModelStateRequest()
    model.model_name = model_name

    pose = get_model_srv(model).pose
    return pose

def movebase_client(move_base_topic, frame_id, pose_xy_heading):
    ### pose_xy_heading: [x, y, heading]

    client = actionlib.SimpleActionClient(move_base_topic, MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_id
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = pose_xy_heading[0]
    goal.target_pose.pose.position.y = pose_xy_heading[1]
    quat = get_quaternion_from_euler(0, 0, pose_xy_heading[2])
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    rospy.init_node('collab_move_test', disable_signals=True)
    spot_amcl_pose_sub = rospy.Subscriber("/spot/amcl_pose", PoseWithCovarianceStamped, spot_amcl_pose_callback)
    for _ in range(10):
        ### Goal1
        result = movebase_client("/spot/move_base", "spot/2d_map", spot_amcl_goal1) 
        pose = get_gazebo_pose("spot")
        print("Goal1:")
        print("spot gazebo xyz:", [pose.position.x, pose.position.y, pose.position.z])
        print("spot gazebo heading:", euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2])
        print("gazebo pose-goal dist, angle:", dist_check(pose, spot_gazebo_goal1))
        time.sleep(1.0)
        ### Goal2        
        result = movebase_client("/spot/move_base", "spot/2d_map", spot_amcl_goal2) 
        pose = get_gazebo_pose("spot")
        print("Goal2:")
        print("spot gazebo xyz:", [pose.position.x, pose.position.y, pose.position.z])
        print("spot gazebo heading:", euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2])
        print("gazebo pose-goal dist, angle:", dist_check(pose, spot_gazebo_goal2))
        time.sleep(1.0)

    exit()

    # rate = rospy.Rate(1)
    
    # while True:
    #     pose = get_gazebo_pose("spot")
    #     print("spot gazebo xyz:", [pose.position.x, pose.position.y, pose.position.z])
    #     print("spot gazebo heading:", euler_from_quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)[2])
    #     # print("spot amcl xyz: ", [spot_pose.position.x - gazebo_amcl_offset[0], spot_pose.position.y - gazebo_amcl_offset[1], spot_pose.position.z])
    #     # print("spot amcl heading: ", euler_from_quaternion(spot_pose.orientation.x, spot_pose.orientation.y, spot_pose.orientation.z, spot_pose.orientation.w)[2])
    #     # print("offset:", [spot_pose.position.x-pose.position.x, spot_pose.position.y-pose.position.y])
    #     rate.sleep()