#!/bin/bash
# IKTypes: http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types

python3 /home/shutter/Programs/openrave/lib/python3.x/site-packages/openravepy/_openravepy_/ikfast.py --robot=shutter.v.0.1.openrave.xml --iktype=Lookat3D --baselink=1 --eelink=6 --savefile=shutter_lookat3d_ikfast61.cpp
