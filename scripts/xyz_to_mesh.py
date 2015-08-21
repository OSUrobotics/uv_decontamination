#!/usr/bin/env python
# Patrick Hansen
# Summer 2015
# xyz_to_mesh.py : reads a raw XYZ file, feeds it into MeshLab, executes
#                  the MLX script (MeshLab script), returns a PLY mesh and
#                  the corresponding new XYZ file (much less dense than raw)

from subprocess import call
import rospkg

# Package path isn't used at the moment, /tmp is, could easily be swapped
rospack = rospkg.RosPack()
packpath = rospack.get_path('uv_decontamination')
filepath = "/tmp"
file1input = filepath + "/etu_points_raw.xyz"
file1output = filepath + "/etu_points_mesh.ply"
file1script = packpath + "/scripts/xyz_to_mesh.mlx"
file2input = file1output
file2output = filepath + "/etu_points_mesh.xyz"

call(["/usr/bin/meshlabserver", "-i", file1input, "-o", file1output, "-s", file1script])

call(["/usr/bin/meshlabserver", "-i", file2input, "-o", file2output])
