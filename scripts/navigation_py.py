#!/usr/bin/env python

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

waypoints = [
    [(-2.136,1.0208,-0.005), (0,0,0,1)],
    [(-1.956,-1.543,-0.009), (0,0,0,1)]
]

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = "map"
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

if __name__ == "__main__":
    rospy.init_node("navigation_py")

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    for pose in waypoints:
        goal = goal_pose(pose)
        client.send_goal(goal)
        client.wait_for_result()
