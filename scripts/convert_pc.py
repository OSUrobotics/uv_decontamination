#!/usr/bin/env python

# Patrick Hansen
# Summer 2015
# convert_pc.py : reads a ROS topic that publishes PointCloud2
#                 messages and converts it into an XYZ file

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from uv_decontamination.msg import XYZ
from uv_decontamination.msg import XYZ_group
from subprocess import call
import rospkg

class pc_converter():

    # This callback handles a standard xyz point cloud
    # It also has the capability to publish on a new topic
    # with the custom XYZ messages type, which is more readable
    # than a PointCloud2 message
    def pc_cb(self, data):
        # point_cloud2 library's read_points function returns a generator
        # where each entry is a list of the fields specified
        gen = pc2.read_points(data, field_names=("x", "y", "z"))

        # Filepath is just /tmp right now
        file_to_open = self.filepath + '/etu_points_raw.xyz'
        f = open(file_to_open, 'w')

        for xyz in gen:

            # This creates the message to be added to the group
            # and eventually published
            point = XYZ()
            point.x = xyz[0]
            point.y = xyz[1]
            point.z = xyz[2]
            self.group.append(point)

            # This is for writing directly to an XYZ file
            to_write = '%(1)f %(2)f %(3)f\n' % {'1':xyz[0], '2':xyz[1], '3':xyz[2]}
            f.write(to_write)

        f.close()

        # This takes the first published point cloud and sends it to
        # the xyz_to_mesh.py script
        if not self.printed:
            self.printed = True
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('uv_decontamination')
            full_path = package_path + '/scripts/xyz_to_mesh.py'
            call(["python", full_path])

    # This callback handles a PointCloud2 that contains a 4th coordinate for
    # contamination level. Only one of these point clouds should be used
    # because they write to the same XYZ files
    def pc_cb_conc(self, data):
        
        gen = pc2.read_points(data, field_names=("x", "y", "z", "contam"))
        file_to_open = self.filepath + '/etu_points_raw.xyz'
        f = open(file_to_open, 'w')

        # Store the concentrations in a different, parallel file
        g = open("/tmp/concentrations.xyz", "w")
        for xyz in gen:
            to_write = '%(1)f %(2)f %(3)f\n' % {'1':xyz[0], '2':xyz[1], '3':xyz[2]}  
            f.write(to_write)
            g.write(str(xyz[3]) +"\n")
        f.close()
        g.close()

        if not self.printed:
            self.printed = True
            rospack = rospkg.RosPack()
            package_path = rospack.get_path('uv_decontamination')
            full_path = package_path + '/scripts/xyz_to_mesh.py'
            call(["python", full_path])
                
    def __init__(self):

        self.filepath = "/tmp"
        
        rospy.init_node('pointcloud_converter', anonymous=True)
        rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, self.pc_cb)
        #rospy.Subscriber("/contamination_points", PointCloud2, self.pc_cb_conc)

        self.pub = rospy.Publisher("octomap_pointcloud", XYZ_group, queue_size=10)
        self.rate = rospy.Rate(1)
        self.group = []
        self.printed = False

if __name__ == '__main__':
    pcc = pc_converter()
    while not rospy.is_shutdown():
        #pcc.pub.publish(pcc.group)
        pcc.rate.sleep()
