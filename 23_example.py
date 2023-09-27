#!/usr/bin/env python
"""
@author Jesse Haviland
"""

import swift
import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import numpy as np

env = swift.Swift()
env.launch(realtime=True)

env.set_camera_pose([1.4, 0, 0.7], [0, 0.0, 0.5])


lTep = (
    sm.SE3.Tx(0.45)
    * sm.SE3.Ty(0.25)
    * sm.SE3.Tz(0.3)
    * sm.SE3.Rx(np.pi)
    * sm.SE3.Rz(np.pi / 2)
)

rTep = (
    sm.SE3.Tx(0.45)
    * sm.SE3.Ty(-0.3)
    * sm.SE3.Tz(0.3)
    * sm.SE3.Rx(np.pi)
    * sm.SE3.Rz(np.pi / 2)
    * sm.SE3.Rx(np.pi / 5)
)

l_target = sg.Sphere(0.01, color=[0.2, 0.4, 0.65, 0.5], base=lTep)
l_target_frame = sg.Axes(0.1, base=lTep)

r_target = sg.Sphere(0.01, color=[0.64, 0.4, 0.2, 0.5], base=rTep)
r_target_frame = sg.Axes(0.1, base=rTep)

env.add(l_target)
env.add(l_target_frame)
env.add(r_target)
env.add(r_target_frame)

print(rTep)
print(lTep)


dt = 0.05


while True:
    # e = np.linalg.pinv(l_target.T) * r_target.T
    # print(e)
    env.step(dt)

env.hold()