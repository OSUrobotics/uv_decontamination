#!/usr/bin/env python

# Patrick Hansen
# Summer 2015
# occlusion.py : Takes a mesh, determines the 

from subprocess import call
import numpy as np
import math
import scipy.optimize as optimize
import glob
from uv_decontamination.msg import uv_coverage
import rospy

POWER = 12 # In watts
TARGET_ENERGY = 40000 # Uniform energy target since concentrations don't work at the moment

# Create a MeshLab script to find visible points from the given x, y point
def make_mlx(name, x, y):
    f = open(name, 'w')

    f.write('<!DOCTYPE FilterScript>\n')
    f.write('<FilterScript>\n')
    f.write('<filter name="Select Visible Points">\n')
    f.write('<Param type="RichDynamicFloat" value="1" min="0" name="radiusThreshold" max="7"/>\n')
    f.write('<Param type="RichBool" value="false" name="usecamera"/>\n')

    # This is the key line. Sets the point.
    f.write('<Param x="%(1)f" y="%(2)f" z="0.5" type="RichPoint3f" name="viewpoint"/>\n' % {'1':x, '2':y})
    
    f.write('<Param type="RichBool" value="false" name="convex_hullFP"/>\n<Param type="RichBool" value="false" name="triangVP"/>\n<Param type="RichBool" value="false" name="reorient"/>\n</filter>\n<filter name="Invert Selection">\n<Param type="RichBool" value="false" name="InvFaces"/>\n<Param type="RichBool" value="true" name="InvVerts"/>\n</filter>\n<filter name="Delete Selected Vertices"/>\n')
    f.write('</FilterScript>\n')

    f.close()
    return name

# This function is useless for now because of a problem bridging the gap
# between the points w/ concentrations and the MeshLab output points
def get_concentrations():
    conc = []
    flist = []
    glist = []
    hlist = []

    final_conc = []
    final_points = []
    
    f = open("/tmp/concentrations.xyz", "r")
    g = open("/tmp/etu_points_mesh.xyz", "r")
    h = open("/tmp/etu_points_raw.xyz", "r")

    for line in f:
        flist.append(float(line))
        
    
    for line in g:
        entry = [float(i) for i in line.split()]
        glist.append(entry)
    g.close()

    for line in h:
        entry = [float(i) for i in line.split()]
        hlist.append(entry)
    h.close()

    print str(len(flist)) + "f lenn"
    print str(len(glist)) + "g lenn"
    print str(len(hlist)) + "h lenn"
    cc = 0.0
    count = 0.0
    dount = 0
    
    for gp in glist:
        if len(gp) != 3:
            continue
        c = 0
        count += 1.0
        matches = []
        for hp in hlist:
            if len(hp) != 3:
                continue
            dist = ((hp[0] - gp[0])**2 + (hp[1] - gp[1])**2 + (hp[2] - gp[2])**2)**0.5
            if dist < 0.1:
                c +=1
                matches.append(hp)
        if c == 0:
            dount += 1
        cc += float(c)

        # Calculate the average concentration
        if len(matches) != 0:
            total = 0.0
            
            for match in matches:
                total += flist[hlist.index(match)]
            avg = total/float(len(matches))    
            final_conc.append(avg)
            final_points.append(gp)
            
    print 'Avg close points = ' + str(cc/count)
    print 'dount ' + str(dount)
    print 'len conc ' + str(len(final_conc))
    print 'len poin ' + str(len(final_points))
    f.close()
    return conc


    
# Get a list of all the points for matching later
def get_full_list():
    strings = []

    f = open("/tmp/etu_points_mesh.xyz", "r")
    for line in f:
        entry = [float(i) for i in line.split()]
        strings.append(entry)
    f.close()
    return strings
    
def calculate_density(distance, angle):
    # Formula that takes a distance and angle of incidence
    # and returns an energy density in microWatts/cm^2
    value = math.cos(angle)*POWER*10**(6)/(4*math.pi*(distance*100)**2)
    return value
    
if __name__ == "__main__":

    # List to store possible poses
    points = []

    # Populate list of possible poses
    with open("/tmp/possible_poses.xyz", "r") as f:
        for line in f:
            coords = [float(i) for i in line.split()]
            point = []
            point.append(coords[0])
            point.append(coords[1])
            points.append(point)

    # For each possible pose, calculate the visible points
    for point in points:
        output_name = '/tmp/visible_points_%(1)f%(2)f.xyz' % {'1':point[0], '2':point[1]}
        output_depth_name = '/tmp/dpth_%(1)f_%(2)f.xyz' % {'1':point[0], '2':point[1]} 
        script_name = '/tmp/visible_points_%(1)f%(2)f.mlx' % {'1':point[0], '2':point[1]}
        script = make_mlx(script_name, point[0], point[1])

        # Call the mlx meshlab script for a given point on the PLY
        call(["/usr/bin/meshlabserver", "-i", "/tmp/etu_points_mesh.ply", "-o", output_name, "-s", script, "-om", "vn"])

        # For each set of visible points, calculate the depth from the point
        with open(output_name, "r") as f, open(output_depth_name, "w") as w:
            for line in f:
                prefix = [float(i) for i in line.split()]
                if len(prefix) != 6:
                    continue

                # This stores the current point to check for depth
                target_point = prefix[:3]

                # Add a z component to the point (x,y)
                point.append(0.5)

                # This is where the UV is coming from
                light_point = point

                # The second half of the XYZ line is the normal
                target_normal = prefix[-3:]

                # Calculate distance from one point to another
                suffix_dep = ((prefix[0] - point[0])**2 + (prefix[1] - point[1])**2 + (prefix[2] - 0.5)**2)**0.5
                difference = [a - b for a,b in zip(target_point, light_point)]
                dot_product = np.dot(difference, target_normal)
                ratio = dot_product/suffix_dep
                suffix_dir = math.acos(ratio)

                # The new point with (x, y, z, depth, direction)
                catted = '%(x)f %(y)f %(z)f %(dep)f %(dir)f\n' % {'x':prefix[0], 'y':prefix[1], 'z':prefix[2], 'dep':suffix_dep, 'dir':suffix_dir}
                w.write(catted)

    # At this point, we have a "dpth" file for each possible pose that stores
    # all of the visible points from that pose along with the points' depths and directions

    all_points = get_full_list()
    #concentrations = get_concentrations()
        
    # Get pointers to all of the "dpth" files
    paths = glob.glob("/tmp/dpth*")
    
    coeff = [[0 for x in range(len(paths))] for x in range(len(all_points))]
    
    for path in paths:
        with open(path, "r") as f:
            for line in f:
                arr = [float(i) for i in line.split()]
                point = arr[:3]
                if point in all_points:
                    coeff[all_points.index(point)][paths.index(path)] = calculate_density(arr[3], arr[4])
        
    call("rm /tmp/visible_points*", shell=True)
    call("rm /tmp/dpth*", shell=True)
    
    # Perform regression
    b = [TARGET_ENERGY] * len(all_points)

    x = optimize.nnls(coeff, b)
    guess = x[0]

    print x

        
    # Stats
    result = np.dot(coeff, guess)
    result_sum = 0
    result_0 = 0
    result_count = 0
    result_max = 0
    total_time = 0
    points_used = 0
    avg_energy = 0
    
    for weight in x[0]:
        total_time += weight
        if weight > 1:
            points_used += 1

    print result
            
    with open('/tmp/hit_points.xyz', 'w') as h, open('/tmp/missed_points.xyz', 'w') as m:
        for index, item in enumerate(result):
            if item == 0:
                result_0 += 1
                #m.write(base_xyz[index])
            else:
                #h.write(base_xyz[index])
                if item > result_max:
                    result_max = item
                    result_max_index = result_0 + result_count
                result_count += 1
                result_sum += item

    avg_energy = result_sum/result_count
    log_kill = 0
    log_percent = 0
    
    if avg_energy > 13000:
        log_kill = 4
        log_percent = 99.99
    elif avg_energy > 9800:
        log_kill = 3
        log_percent = 99.9
    elif avg_energy > 6600:
        log_kill = 2
        log_percent = 99
    elif avg_energy > 3400:
        log_kill = 1
        log_percent = 90

    call(["clear"])

    
    pub = rospy.Publisher("/uv_results", uv_coverage, queue_size=10)
    rospy.init_node("occlusion_result")
    r = rospy.Rate(0.5)

    msg = uv_coverage()
    msg.log_kill = log_kill
    msg.minutes = int(total_time/60)
    msg.seconds = int(total_time%60)
    msg.avg_energy = avg_energy
    msg.num_points = points_used

    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()
