#!/usr/bin/env python

import sys
import rospy
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import moveit_commander
import math
import numpy as np
import copy

# Initialize moveit_commander and rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_stretch", anonymous=True)

# Instantiate a RobotCommander object. Provides information on the robot's
# kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()

# Instantiate a PlanningSceneInterface object. Provides remote interface for
# getting, setting and updating the robot's internal understanding of the
# surrounding world
scene = moveit_commander.PlanningSceneInterface()

# Instantiate a MoveGroupCommander object. It's an interface to a planning group
group_name="stretch_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)

# Start DisplayTrajectory Publisher. Used to display the trajectory in Rviz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)



# Getting basic information
planning_frame = move_group.get_planning_frame()  # Reference frame for the robot
print("============ Planning frame: %s" % planning_frame)

eef_link = move_group.get_end_effector_link()  # Get name of end-effector link
print("============ End-effector link: %s" % eef_link)

group_names = robot.get_group_names()  # Get list of all groups in the robot
print("============ Available Planning Groups:", group_names)

print("============ Printing robot state")  # Print all joint states of the robot
print(robot.get_current_state())
print("")

print("============ Printing current end effector pose")  # Print all joint states of the robot
print(move_group.get_current_pose())
print("")

num_waypoints = 11

z_min = 0.29
z_max = 1.29
y_min = -0.65
y_max = -0.15

amplitude = (y_max - y_min) / 2
y_offset = y_min + amplitude


z_range = np.linspace(z_min, z_max, num_waypoints)
y_range = -amplitude * np.cos( 2*math.pi*(z_range - z_min) ) + y_offset

current_pose = move_group.get_current_pose().pose

waypoints = []
for i in range(num_waypoints):
    goal_pose = copy.deepcopy(current_pose)
    goal_pose.position.y = y_range[i]
    goal_pose.position.z = z_range[i]

    waypoints.append(goal_pose)

waypoints_reversed = waypoints[::-1]
waypoints = waypoints + waypoints_reversed
print(len(waypoints))

(plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0) 

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
# Publish
display_trajectory_publisher.publish(display_trajectory);


# move_group.execute(plan, wait=True)
# # plan = move_group.go(wait=True)
# move_group.stop()

# Good practice to clear targets after planning with poses
# move_group.clear_pose_targets()




