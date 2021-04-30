#!/usr/bin/env python3

from stretch_collaboration.msg import HeadScan, MaxHeightImage, VolumeOfInterest
from geometry_msgs.msg import Vector3
rom stretch_funmap import mapping as ma

def voi_to_msg(voi):
    voi_msg = VolumeOfInterest()
    voi_msg.axes = voi['axes']
    voi_msg.frame_id = voi['frame_id']
    
    voi_msg.origin.x = voi['origin'][0]
    voi_msg.origin.y = voi['origin'][1]
    voi_msg.origin.z = voi['origin'][2]

    voi_msg.in_m.x = voi['in_m'][0]
    voi_msg.in_m.y = voi['in_m'][1]
    voi_msg.in_m.z = voi['in_m'][2]

    return voi_msg

def max_height_image_to_msg(max_height_im):
    mhi_msg = MaxHeightImage()

    voi_msg = voi_to_msg(max_height_im.voi)
    mhi_msg.voi = voi_msg

    mhi_msg.image = max_height_im.image
    mhi_msg.rgb_image = max_height_im.rgb_image
    mhi_msg.camera_depth_image = max_height_im.camera_depth_image
    # Check if None
    mhi_msg.transform_original_to_corrected = max_height_im.transform_original_to_corrected
    mhi_msg.transform_corrected_to_original = max_height_im.transform_corrected_to_original
    mhi_msg.m_per_pix = max_height_im.m_per_pix
    mhi_msg.m_per_height = max_height_im.m_per_height

    mhi_msg.image_origin.x = max_height_im.image_origin[0]
    mhi_msg.image_origin.y = max_height_im.image_origin[1]
    mhi_msg.image_origin.z = max_height_im.image_origin[2]

    return mhi_msg


def headscan_to_msg(head_scan):
    hs_msg = HeadScan()
    
    # Pack max height image into a message
    mhi_msg = max_height_image_to_msg(head_scan.max_height_im)
    hs_msg.max_height_im = mhi_msg
    
    hs_msg.robot_xy_pix = head_scan.robot_xy_pix 
    hs_msg.robot_ang_rad = head_scan.robot_ang_rad  # Check if list or not
    hs_msg.timestamp = head_scan.timestamp
    hs_msg.base_link_to_image_mat = head_scan.base_link_to_image_mat
    hs_msg.base_link_to_map_mat = head_scan.base_link_to_map_mat
    hs_msg.image_to_map_mat = head_scan.image_to_map_mat
    hs_msg.image_to_base_link_mat = head_scan.image_to_base_link_mat
    hs_msg.map_to_image_mat = head_scan.map_to_image_mat
    hs_msg.map_to_base_link_mat = head_scan.map_to_image_mat 

    return hs_msg
    


