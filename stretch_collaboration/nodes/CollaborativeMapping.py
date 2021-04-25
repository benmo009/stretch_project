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
        head_scan.execute_full(node)

        self.load_headscan_to_param(head_scan)


    def main(self):
        # Initialize nodes, publishers, subscribers, services
        hm.HelloNode.main(self, 'collab_funmap', 'collab_funmap')

    
    def load_headscan_to_param(self, headscan):
        # Load HeadScan data
        if "tolist" in dir(headscan.robot_ang_rad):
            robot_ang_rad = headscan.robot_ang_rad.tolist()
        else:
            robot_ang_rad = headscan.robot_ang_rad
        hs_data = {'robot_xy_pix' : headscan.robot_xy_pix.tolist(),
                'robot_ang_rad' : robot_ang_rad,
                'timestamp' : {'secs':headscan.timestamp.secs, 'nsecs':headscan.timestamp.nsecs},
                'base_link_to_image_mat' : headscan.base_link_to_image_mat.tolist(), 
                'base_link_to_map_mat' : headscan.base_link_to_map_mat.tolist(), 
                'image_to_map_mat' : headscan.image_to_map_mat.tolist(), 
                'image_to_base_link_mat' : headscan.image_to_base_link_mat.tolist(), 
                'map_to_image_mat' : headscan.map_to_image_mat.tolist(), 
                'map_to_base_mat' : headscan.map_to_base_mat.tolist()}

        rospy.set_param("stretch_re1_1027/HeadScan", hs_data)

        # Load maxHeightImage data
        mhi = headscan.max_height_im
        rgb_im = mhi.rgb_image.tolist()
        cam_depth_im = mhi.camera_depth_image.tolist()
        im = mhi.image.tolist()

        voi_data = mhi.voi.serialize()
        voi_data['origin'] = voi_data['origin'].tolist()
        voi_data['axes'] = voi_data['axes'].tolist()

        if mhi.transform_original_to_corrected is not None:
            transform_original_to_corrected = mhi.transform_original_to_corrected.tolist()
        else:
            transform_original_to_corrected = None

        if mhi.transform_corrected_to_original is not None:
            transform_corrected_to_original = mhi.transform_corrected_to_original.tolist()
        else:
            transform_corrected_to_original = None
        
        mha_data = {'image.dtype': str(mhi.image.dtype),
                'image.shape': list(mhi.image.shape),
                'np.max(image)': max_pix, 
                'm_per_pix': mhi.m_per_pix,
                'm_per_height_unit': mhi.m_per_height_unit,
                'voi_data': voi_data,
                'image_origin': mhi.image_origin.tolist(),
                'transform_original_to_corrected': transform_original_to_corrected, 
                'transform_corrected_to_original': transform_corrected_to_original
        }

        rospy.set_param("stretch_re1_1027/MaxHeightImage", mhi_data)
        rospy.set_param("stretch_re1_1027/MaxHeightImage/image", im)
        rospy.set_param("stretch_re1_1027/MaxHeightImage/rgb_image", rgb_im)
        rospy.set_param("stretch_re1_1027/MaxHeightImage/camera_depth_image", cam_depth_im)


    def get_headscan_from_param(self):
        pass

if __name__ == "__main__":
    colab_map = FunmapNode()
    colab_map.main()
    colab_map.perform_head_scan()