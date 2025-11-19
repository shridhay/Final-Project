#!/usr/bin/env python3

# Script to generate IKFast cpp for shutter. This script can be used instead
# of calling directly ikfast.py:
#
# python /home/shutter/Programs/openrave/lib/python3.x/site-packages/openravepy/_openravepy_/ikfast.py \
#   --robot=shutter.v.0.1.openrave.xml --iktype=Lookat3D --baselink=1 --eelink=6 --savefile=shutter_lookat3d_ikfast61.cpp
#
# IKTypes can be found in http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types
#
import sys

import openravepy as orpy
from openravepy import ikfast

import faulthandler
faulthandler.enable()

robot_model = sys.argv[1]
print("Genering IKFast for {}".format(robot_model))

env = orpy.Environment()
kinbody = env.ReadRobotXMLFile(robot_model)
env.Add(kinbody)
solver = ikfast.IKFastSolver(kinbody=kinbody)

chaintree = solver.generateIkSolver(baselink=1,
                                    eelink=4,
                                    freeindices=[],
                                    solvefn=ikfast.IKFastSolver.solveFullIK_Translation3D)
print("generated solver")
code = solver.writeIkSolver(chaintree)
print("generated code")

with open('ik.cpp','w') as fid:
    fid.write(code)

print("wrote code. done.")
