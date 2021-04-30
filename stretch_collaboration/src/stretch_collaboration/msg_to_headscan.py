#!/usr/bin/env python

from stretch_collaboration.msg import HeadScan, MaxHeightImage, VolumeOfInterest
import stretch_funmap.mapping as ma
import stretch_funmap.max_height_image as mhi

def msg_to_volume_of_interest(voi_msg):
    # frame_id = voi_msg.frame_id
    # axes = voi_msg.axes 
    origin = voi_msg.origin 
    origin = np.array([origin.x, origin.y, origin.z])
    # x_in_m = voi_msg.in_m.x
    # y_in_m = voi_msg.in_m.y
    # z_in_m = voi_msg.in_m.z

    voi = mhi.VolumeOfInterest(frame_id=voi_msg.frame_id,
                               origin=origin, 
                               axes=voi_msg.axes,
                               x_in_m=voi_msg.in_m.x, 
                               y_in_m=voi_msg.in_m.y, 
                               z_in_m=voi_msg.in_m.z)

    return voi

def msg_to_max_height_image(mhi_msg):
    voi = msg_to_volume_of_interest(mhi_msg.voi)

    if mhi_obj.camera_depth_image is not None:
        use_camera_depth_image = True
    else:
        use_camera_depth_image = False

    mhi_obj = mhi.MaxHeightImage(volume_of_interest=voi, 
                                 m_per_pix=mhi_msg.m_per_pix, 
                                 pixel_dtype=mhi_msg.image.dtype, 
                                 m_per_height_unit=mhi_msg.m_per_height_unit, 
                                 use_camera_image=use_camera_depth_image, 
                                 image=mhi_msg.image, 
                                 rgb_image=mhi_msg.rgb_image, 
                                 camera_depth_image=mhi_msg.camera_depth_image)
    
    mhi_obj.transform_original_to_corrected = mhi_msg.transform_original_to_corrected
    mhi_obj.transform_corrected_to_original = mhi_msg.transform_corrected_to_original

    return mhi_obj

def mst_to_head_scan(hs_msg):
    max_height_image = msg_to_max_height_image(hs_msg.max_height_im)
    head_scan = ma.HeadScan(max_height_im=max_height_image)
    
    head_scan.robot_xy_pix = hs_msg.robot_xy_pix
    head_scan.robot_ang_rad = hs_msg.robot_ang_rad
    head_scan.timestamp = rospy.Time()
    head_scan.base_link_to_image_mat = hs_msg.base_link_to_image_mat
    head_scan.base_link_to_map_mat = hs_msg.base_link_to_map_mat
    head_scan.image_to_map_mat = hs_msg.image_to_map_mat
    head_scan.image_to_base_link_mat = hs_msg.image_to_base_link_mat
    head_scan.map_to_image_mat = hs_msg.map_to_image_mat
    head_scan.map_to_base_mat = hs_msg.map_to_base_mat

    return head_scan
