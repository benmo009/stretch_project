#!/usr/bin/env python

import rospy
import rosbag
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Pose, Vector3

class HumanLocalizer:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("human_localizer")

        self.rate = rospy.Rate(2)
        self.bag = rosbag.Bag("/home/ben/human_localization.bag", 'w')

        # Initialize MarkerArray subscriber
        self.face_sub = rospy.Subscriber("/faces/marker_array", MarkerArray, self.face_callback)
        self.body_sub = rospy.Subscriber("/body_landmarks/marker_array", MarkerArray, self.body_callback)

        self.marker_pub = rospy.Publisher("/localized_person", Marker, queue_size=0)

        self.face_pose = Pose()
        self.avg_body = Vector3()

        rospy.wait_for_message("/body_landmarks/marker_array", MarkerArray)
        rospy.wait_for_message("/faces/marker_array", MarkerArray)



    def face_callback(self, data):
        if len(data.markers) > 0:
            face_marker = data.markers[0]
            self.face_pose = face_marker.pose
            self.bag.write('face', self.face_pose.position)

        else:
            rospy.loginfo("No face detected")
            
    
    def body_callback(self, data):
        # Take each point position and average them out
        if len(data.markers) > 0:
            points = data.markers[0].points 
            avg_point = Vector3()

            # Add check for length of points 
            for point in points:
                avg_point.x += point.x 
                avg_point.y += point.y 
                avg_point.z += point.z 

            

            avg_point.x /= len(points)
            avg_point.y /= len(points)
            avg_point.z /= len(points)

            marker = Marker()
            marker.pose.orientation.w = 1
            marker.header.frame_id = data.markers[0].header.frame_id
            marker.pose.position=avg_point
            marker.color.r = 1
            marker.color.a = 1
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.type=1
            
            self.body_pose = avg_point
            self.bag.write('body', self.body_pose)

            self.marker_pub.publish(marker)
            rospy.loginfo(avg_point)

        else:
            rospy.loginfo("No body landmarks detected")
            


    def main(self):
        while not rospy.is_shutdown():
            rospy.loginfo("Face position - x: {:.3f} y: {:.3f} z: {:.3f}".format(
                                                    self.face_pose.position.x, 
                                                    self.face_pose.position.y, 
                                                    self.face_pose.position.z))

            rospy.loginfo("Body position - x: {:.3f} y: {:.3f} z: {:.3f}".format(
                                                    self.body_pose.x, 
                                                    self.body_pose.y, 
                                                    self.body_pose.z))
            self.rate.sleep()
        #rospy.spin()


if __name__ == "__main__":
    hl_node = HumanLocalizer()
    hl_node.main()

