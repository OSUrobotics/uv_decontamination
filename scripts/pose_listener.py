#!/usr/bin/env python

# Patrick Hansen
# Summer 2015
# pose_listener.py : listens to the robots pose and goals. Waits to
#                    for robot to get within tolerance of goal to decide
#                    if it's complete. (There's no "GOAL_COMPLETE" message
#                    to my knowledge.) Listens for as many goals as its told.

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped as PoseMsg
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from subprocess import call
import rospkg

class PathTracker():
    def __init__(self):
        self.points = []
        self.count = 0
        self.goal = None

    # Add the pose to our list of poses
    def pose_cb(self, data):
        point = {}
        point['x'] = data.pose.pose.position.x
        point['y'] = data.pose.pose.position.y
        self.points.append(point)
        
        self.count += 1

    # Store the goal in the object
    def goal_cb(self, data):
        goal = {}
        goal['x'] = data.pose.position.x
        goal['y'] = data.pose.position.y
        self.goal = goal

    # Just before this node is shutdown, write to possible_poses.xyz,
    # which will be used by occlusion.py
    def final_msg(self):
        self.trim_points()
        with open("/tmp/possible_poses.xyz", "w") as f:
            for point in self.points:
                if point != None:
                    line = "%(x)f %(y)f\n" % point
                    f.write(line)
                    
    # Weed out points that are really close together
    # The result is a pretty nice spacing of points
    # Tolerance can be adjusted as necessary
    def trim_points(self):
        trimmed = []
        tolerance = 0.1
        trimmed.append(self.points[0])
        recentPoint = self.points[0]
        for point in self.points:
            xdif = abs(point['x'] - recentPoint['x'])
            ydif = abs(point['y'] - recentPoint['y'])
            if (xdif + ydif) > tolerance:
                recentPoint = point
                trimmed.append(point)

        self.plot_points(trimmed, self.points)
        self.points = trimmed

    # Used for visualization of the path and selected points
    def plot_points(self, l1, l2):
        l1x = []
        l1y = []
        l2x = []
        l2y = []
        for point in l1:
            l1x.append(point['x'])
            l1y.append(point['y'])
        for point in l2:
            l2x.append(point['x'])
            l2y.append(point['y'])

        plt.plot(l1x, l1y, 'rs', l2x, l2y, 'b--')
        #plt.show()

# Check if a goal is close enough
def is_within_tolerance(point, goal):
    xtol = abs(point['x'] - goal['x'])
    ytol = abs(point['y'] - goal['y'])
    
    if xtol < 0.2 and ytol < 0.2:
        return True
    else:
        return False
    
if __name__ == '__main__':

    pt = PathTracker()
    
    rospy.init_node('pose_listener', anonymous=True)
    rospy.on_shutdown(pt.final_msg)
    poseSubscriber = rospy.Subscriber("amcl_pose", PoseMsg, pt.pose_cb)
    goalSubscriber = rospy.Subscriber("/move_base/current_goal", PoseStamped, pt.goal_cb)
    # Set default goal count in case it can't get param
    goal_count = 2
    goal_count = rospy.get_param("/pose_listener/num_goals")
    count = 0
    print goal_count
    
    while not rospy.is_shutdown():
        if len(pt.points) > 0 and pt.goal != None:
            if is_within_tolerance(pt.points[-1], pt.goal):
                count += 1
                print count

            # As soon as we finish, run occlusion.py
            if count == goal_count:
                rospy.signal_shutdown("Reached max point count")
                rospack = rospkg.RosPack()
                package_path = rospack.get_path('uv_decontamination')
                full_path = package_path + "/scripts/occlusion.py"
                call(full_path, shell=True)
