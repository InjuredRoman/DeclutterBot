#!/usr/bin/python3
import numpy as np
from autolab_core import RigidTransform
from frankapy import FrankaArm
import time

objects = [[ 0.26984675,  0.085695,   -0.16977288, -2.62005117, -0.03062543,  2.71013476,
  1.03442248],[ 0.34283297,  0.41885565,  0.06461629, -2.53731683, -0.03329968,  2.97190837, 2.73442564],
    [ 2.61825285e-02,  7.17213862e-01,  2.16404466e-02, -1.86661080e+00, -8.12655016e-04,  2.54255666e+00,  7.66709999e-01]]

final_box = [ 0.07525657,  0.04672782, -0.46091937, -2.5029303,  -0.03062509,  2.59145016, 0.67573284]


home = [ 1.96596364e-04, -7.85841667e-01, -3.06014654e-03, -2.35654641e+00, -4.62090277e-04,  1.57150903e+00,  7.85095747e-01]

fa = FrankaArm()

def startup():
  fa.reset_joints()
  fa.open_gripper()
  fa.goto_joints(home)

def go_pose(joint):
    fa.goto_joints(joint)
   
def gripper(state):
    if(state=='open'):
      fa.open_gripper()
    else:
       fa.close_gripper()

if __name__ == "__main__":
    startup()
    
    for obj in objects:
        go_pose(home)
        go_pose(obj)

        gripper('close')

        go_pose(home)
        go_pose(final_box)
        gripper('open')

    go_pose(home)
    print("----------GOOD JOB COMRADE --------")
