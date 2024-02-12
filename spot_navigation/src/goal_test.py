#!/usr/bin/env python3
# coding=utf8

import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseWithCovarianceStamped

def haetae_callback(haetae_msg):
    global haetae_xyz
    haetae_xyz = haetae_msg.pose.pose.position
    print("haetae : ", haetae_xyz.x, haetae_xyz.y, haetae_xyz.z)

pub1 = rospy.Publisher('/haetae/move_base/goal', MoveBaseActionGoal, queue_size=10)
#pub2 = rospy.Publisher('/spot/move_base/goal', MoveBaseActionGoal, queue_size=10)
#feedback_sub2 = rospy.Subscriber("/haetae/amcl_pose", PoseWithCovarianceStamped, haetae_callback)

rospy.init_node('goal_test')

picking_station1_pos = [2.77, -1.8, 0.49]
picking_station1_ori = [0,0,-0.7071068,0.7071068]
picking_station1_parking_pos = [0, -0.4, 0, 0, 0, np.pi/2]
picking_station2_pos = [2.77, 0, 0.49]
picking_station2_ori = [0,0,-0.7071068,0.7071068]
picking_station2_parking_pos = [0, -0.4, 0, 0, 0, np.pi/2]

placing_shelf1_pos = [-2.2, -1.65, 0.49]
placing_shelf1_ori = [0,0,0.7071068,0.7071068]
placing_shelf1_parking_pos = [0, -0.4, 0, 0, 0, np.pi/2]
placing_shelf2_pos = [-2.2, -0.675, 0.49]
placing_shelf2_ori = [0,0,0.7071068,0.7071068]
placing_shelf2_parking_pos = [0, -0.4, 0, 0, 0, np.pi/2]
placing_shelf3_pos = [-2.2, 0.3, 0.49]
placing_shelf3_ori = [0,0,0.7071068,0.7071068]
placing_shelf3_parking_pos = [0, -0.4, 0, 0, 0, np.pi/2]

haetae_msg=MoveBaseActionGoal()
#spot_msg=MoveBaseActionGoal()

haetae_msg.goal.target_pose.header.frame_id = 'haetae_map'
haetae_msg.goal.target_pose.pose.position.x =  1.0 #picking_station1_pos[0] + picking_station1_parking_pos[0]
haetae_msg.goal.target_pose.pose.position.y =  -1.8 #picking_station1_pos[1] + picking_station1_parking_pos[1]
haetae_msg.goal.target_pose.pose.position.z =  0.0 #picking_station1_pos[2] + picking_station1_parking_pos[2]
haetae_msg.goal.target_pose.pose.orientation.x = 0 #picking_station1_ori[0]
haetae_msg.goal.target_pose.pose.orientation.y = 0#picking_station1_ori[1] 
haetae_msg.goal.target_pose.pose.orientation.z = 0#picking_station1_ori[2] 
haetae_msg.goal.target_pose.pose.orientation.w = 0#picking_station1_ori[3] 
    
'''    
spot_msg.goal.target_pose.header.frame_id = 'spot/2d_map'
spot_msg.goal.target_pose.pose.position.x = spot_pos_list[i%2][0]
spot_msg.goal.target_pose.pose.position.y = spot_pos_list[i%2][1]
spot_msg.goal.target_pose.pose.position.z = spot_pos_list[i%2][2]
spot_msg.goal.target_pose.pose.orientation.x = orient_list[i%2][0]
spot_msg.goal.target_pose.pose.orientation.y = orient_list[i%2][1]
spot_msg.goal.target_pose.pose.orientation.z = orient_list[i%2][2]
spot_msg.goal.target_pose.pose.orientation.w = orient_list[i%2][3]
'''

pub1.publish(haetae_msg)
#pub2.publish(spot_msg)
rospy.sleep(25)