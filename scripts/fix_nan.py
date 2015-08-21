#!/usr/bin/env python

# Patrick Hansen
# Summer 2015
# fix_nan.py : replace the NaN in the 2d map YAML with a 0.0

# There's probably a more elegant way to do this than using subprocess to
# call a sed command in a Python file...

from subprocess import call
import rospkg

# cats can be used to verify output
rospack = rospkg.RosPack()
file_prefix = rospack.get_path("uv_decontamination")
file_name = file_prefix + "/maps/octo_etu_2d.yaml"
script_call = 'sed -i "s/nan/0.000/" ' + file_name
#cat_call = "cat " + file_name
#call(cat_call, shell=True)
call(script_call, shell=True)
#call(cat_call, shell=True)
