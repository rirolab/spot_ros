#!/usr/bin/env python3
# ROS
import rospy, rospkg
from nav_msgs.msg import OccupancyGrid
import numpy as np

class DynamicObjectFiltering:
    def __init__(self):
        self.map_high_res = None
        self.map_low_res = None

        # Publishers
        self.final_map_pub = rospy.Publisher("final_map",OccupancyGrid,queue_size=5,latch=True)    # changed "/final_map" to "final_map"

        # Subscribers
        rospy.Subscriber("projected_map", OccupancyGrid, self.map_high_res_callback)  # changed "/projected_map" to "projected_map"
        rospy.Subscriber("projected_map2", OccupancyGrid, self.map_low_res_callback)  # changed "/projected_map2" to "projected_map2"
        
    def map_high_res_callback(self, msg):
        self.map_high_res = msg

    def map_low_res_callback(self, msg):
        self.map_low_res = msg

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.map_high_res is None or self.map_low_res is None:
                print(self.map_high_res is None, self.map_low_res is None)
                continue

            # Get data
            map_high_res_info = self.map_high_res.info
            map_low_res_info = self.map_low_res.info
            high_res_map = np.array(self.map_high_res.data)
            low_res_map = np.array(self.map_low_res.data)

            ratio = int(map_low_res_info.resolution / map_high_res_info.resolution)
            origin_diff_x = int((map_high_res_info.origin.position.x - map_low_res_info.origin.position.x) / map_high_res_info.resolution)
            origin_diff_y = int((map_high_res_info.origin.position.y - map_low_res_info.origin.position.y) / map_high_res_info.resolution)

            try:
                high_res_map_wh = np.reshape(high_res_map, (map_high_res_info.height, map_high_res_info.width))
            except:
                # Added because error in self.map_high_res.info.width
                print("Map size is inaccurate. Restart program.", len(high_res_map), map_high_res_info.height, map_high_res_info.width)
                modified_high_res_w = len(high_res_map) // map_high_res_info.height
                w_mod = len(high_res_map) % map_high_res_info.height
                if w_mod != 0: high_res_map = high_res_map[:-w_mod]
                high_res_map_wh = np.reshape(high_res_map, (map_high_res_info.height, modified_high_res_w))
            # try:
            low_res_map_wh = np.reshape(low_res_map, (map_low_res_info.height, map_low_res_info.width))
            # except:
            #     # Added because error in self.map_low_res.info.width
            #     print("Map size is inaccurate. Restart program.", len(low_res_map), map_low_res_info.height, map_low_res_info.width)
            #     modified_low_res_w = len(low_res_map) // map_low_res_info.height
            #     w_mod = len(low_res_map) % map_low_res_info.height
            #     if w_mod != 0: low_res_map = low_res_map[:-w_mod]
            #     low_res_map_wh = np.reshape(low_res_map, (map_low_res_info.height, modified_low_res_w))

            low_res_map_resize = np.kron(low_res_map_wh, np.ones((ratio, ratio)))
            if origin_diff_x > 0:
                low_res_map_resize = np.pad(low_res_map_resize, ((0,0),(0, origin_diff_x)), 'constant', constant_values=0)
            elif origin_diff_x < 0:
                low_res_map_resize = low_res_map_resize[:][:origin_diff_x]
            if origin_diff_y > 0:
                low_res_map_resize = np.pad(low_res_map_resize, ((0,0),(origin_diff_y, 0)), 'constant', constant_values=0)
            elif origin_diff_y < 0:
                low_res_map_resize = low_res_map_resize[:origin_diff_y][:]

            new_map_w = min(high_res_map_wh.shape[1], low_res_map_resize.shape[1])
            new_map_h = min(high_res_map_wh.shape[0], low_res_map_resize.shape[0])
            high_res_map_cut = high_res_map_wh[:new_map_h, :new_map_w]
            low_res_map_cut = low_res_map_resize[:new_map_h, :new_map_w]

            final_map = np.where(low_res_map_cut != 0, high_res_map_cut, low_res_map_cut)
            final_map = np.reshape(final_map, new_map_w*new_map_h)

            # Stream results
            pub_map = OccupancyGrid()
            pub_map.header = self.map_high_res.header
            pub_map.info = self.map_high_res.info
            pub_map.info.width = new_map_w
            pub_map.info.height = new_map_h
            pub_map.data = final_map.astype(int).tolist()
            self.final_map_pub.publish(pub_map)

if __name__=="__main__":
    rospy.init_node('dynamic_object_filtering')

    mode = DynamicObjectFiltering()
    mode.run()
