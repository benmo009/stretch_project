#!/usr/bin/env python
from __future__ import print_function

import stretch_body.robot as rb
from stretch_body.hello_utils import ThreadServiceExit

import rospy
from geometry_msgs.msg import WrenchStamped

class StretchForceSensor:
    def __init__(self):
        pass

    ###### PUBLISH FORCES #######

    def get_forces_and_publish(self):
        # Get the force sensor readings
        arm_force = self.robot.arm.status['force'] 
        lift_force = self.robot.lift.status['force']

        curent_time = rospy.Time.now()

        forces = WrenchStamped()
        forces.header.stamp = current_time
        # forces.header.frame_id = 
        forces.wrench.force.z = lift_force
        forces.wrench.force.y = arm_force

        self.force_pub.publish(forces)

    ########### MAIN ############

    def main(self):
        # Initialize ROS Node
        rospy.init_node("stretch_force_sensors")
        self.node_name = rospy.get_name()

        rospy.loginfo("For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.")

        rospy.loginfo("{:s} started".format(self.node_name))

        self.robot = rb.Robot()
        self.robot.startup()

        # Get the rate for publishing the force sensor data
        self.force_sensor_rate = rospy.get_param("~rate", default=15.0)
        #self.force_sensor_frame_id = ""

        self.force_pub = rospy.Publisher('force_sensors', WrenchStamped, queue_size=1)

        rospy.loginfo("{0} rate = {1} Hz".format(self.node_name, self.force_sensor_rate))

        try:
            # Publish force sensor readings until ROS is shutdown
            while not rospy.is_shutdown():
                self.get_forces_and_publish()
                self.force_sensor_rate.sleep()
        except (rospy.ROSInterruptException, ThreadServiceExit):
            self.robot.stop()
            rospy.signal_shutdown("stretch_force_sensors shutdown")


if __name__ == "__main__":
    node = StretchForceSensor()
    node.main()