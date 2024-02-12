#!/usr/bin/env python3

import rospy
import time
from control_msgs.msg import JointTrajectoryControllerState

if __name__ == '__main__':
    ### Init rosnode
    rospy.init_node('wait_sim_start', disable_signals=True)
    ### Wait
    rospy.wait_for_message("joint_group_position_controller/state", JointTrajectoryControllerState, timeout=None) # namespace is applied
    rospy.wait_for_service('/gazebo/set_model_state', timeout=None)
    init_time = rospy.get_time()
    etime = rospy.get_time() - init_time
    while etime < 2.0: # Wait for 2.0 sec in simulation time.
        etime = rospy.get_time() - init_time
        time.sleep(0.1)
    print("Start fastlio_mapping.")
    exit()
