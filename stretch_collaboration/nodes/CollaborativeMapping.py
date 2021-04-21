#!/usr/bin/env python

import rospy

import hello_helpers.hello_misc as hm 

import stretch_funmap.merge_maps as mm 
import stretch_funmap.mapping as ma 


# Initialize node

class FunmapNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

        self.merged_map = None
        self.localized = False

    
    # Perform a scan with the camera
    def perform_head_scan(self):
        node = self

        # Reduce the occlusion due to the arm and grabber. This is 
        # inteded to be run when the standard grabber is not holding 
        # an object.
        ma.stow_and_lower_arm(node)

        # Create and perform a new full scan o f the environment using
        # the head
        head_scan = ma.HeadScan(voi_side_m=16.0)
        head_scan.execute_full(node, fast_scan=fast_scan)

        self.load_headscan_to_param(head_scan)


    def main(self):
        # Initialize nodes, publishers, subscribers, services
        hm.HelloNode.main(self, 'collab_funmap', 'collab_funmap')

    
    def load_headscan_to_param(self, headscan):

        if "tolist" in dir(headscan.robot_ang_rad):
            robot_ang_rad = headscan.robot_ang_rad.tolist()
        else:
            robot_ang_rad = headscan.robot_ang_rad
        data = {'max_height_image_base_filename' : max_height_image_base_filename,
                'robot_xy_pix' : headscan.robot_xy_pix.tolist(),
                'robot_ang_rad' : robot_ang_rad,
                'timestamp' : {'secs':headscan.timestamp.secs, 'nsecs':self.timestamp.nsecs},
                'base_link_to_image_mat' : headscan.base_link_to_image_mat.tolist(), 
                'base_link_to_map_mat' : headscan.base_link_to_map_mat.tolist(), 
                'image_to_map_mat' : headscan.image_to_map_mat.tolist(), 
                'image_to_base_link_mat' : headscan.image_to_base_link_mat.tolist(), 
                'map_to_image_mat' : headscan.map_to_image_mat.tolist(), 
                'map_to_base_mat' : headscan.map_to_base_mat.tolist()}

        rospy.set_param("stretch_re1_2017/map", data)

if __name__ == "__main__":
    colab_map = FunmapNode()
    colab_map.main()
    colab_map.perform_head_scan()