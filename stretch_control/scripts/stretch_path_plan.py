#!/usr/bin/env python

import sys
import rospy
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import moveit_commander
import math

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
group_name="panda_arm" # **ONLY FOR THE TUTORIAL, CHANGE FOR STRETCH LATER**
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


# Plan a Joint Goal
joint_goal = move_group.get_current_joint_values()  # Get list of current joint values
print("============ Current Joint values:", joint_goal)
# joint_goal[0] = 0
# joint_goal[1] = -math.pi/4
# joint_goal[2] = 0
# joint_goal[3] = -math.pi/2
# joint_goal[4] = 0
# joint_goal[5] = math.pi/3
# joint_goal[6] = 0

# print("="*12, "Goal Joint Values:", joint_goal)

# # Move to the joint goal. go can be called w/ joint values, poses, or without
# # any parameters if already set the pose/joint target for the group
# move_group.go(joint_goal, wait=True)

# # Call stop() to ensure there is no residual movement
# move_group.stop()


# Planning a Pose Goal
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.x = 0.3
pose_goal.orientation.y = 0.1
pose_goal.orientation.z = 0.1
pose_goal.orientation.w = 0.5
pose_goal.position.x = 0.5
pose_goal.position.y = 0.5
pose_goal.position.z = 0.5

# Set the pose target
print("="*12, "Setting a pose target:\n", pose_goal)
move_group.set_pose_target(pose_goal)

# Compute the plan and execute it
(success, plan, plan_time, _) = move_group.plan(pose_goal)

if success:
    print("Successfully computed trajectory! Compute in %f seconds" % plan_time)

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)

display_trajectory_publisher.publish(display_trajectory)

move_group.execute(plan)
# plan = move_group.go(wait=True)
# move_group.stop()

# # Good practice to clear targets after planning with poses
# move_group.clear_pose_targets()




