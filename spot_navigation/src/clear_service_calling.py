#!/usr/bin/env python3
# coding=utf8

from std_srvs.srv import Empty
import rospy, sys

def calling_loop():
    robot_name = sys.argv[1]
    while(1):
        rospy.wait_for_service('/%s/move_base/clear_costmaps'%robot_name)
        rospy.loginfo("clear_costmaps service prepared")
        
        try:
            clear_call = rospy.ServiceProxy('/%s/move_base/clear_costmaps'%robot_name, Empty)
            clear_call()
            rospy.loginfo("clear_costmaps service called!")
        except rospy.ServiceException as e:
            rospy.loginfo("clear_costmaps Service call failed: %s"%e)
        rospy.sleep(20)

if __name__ == "__main__":

    calling_loop()
    
    rospy.spin()
